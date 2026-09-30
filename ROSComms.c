#include "ROSComms.h"
#include "global.h"
#include "common.h"
#include "queue.h"
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "task.h"
#include "timers.h"
#include "cJSON.h"
#include "powermgmt.h"
#include <stdarg.h>
#include <math.h>


#define cJSON_GetInt(object, key) (cJSON_GetObjectItem(object, key) ? cJSON_GetObjectItem(object, key)->valueint : 0)
#define cJSON_GetStr(object, key) (cJSON_GetObjectItem(object, key) ? cJSON_GetObjectItem(object, key)->valuestring : "")
//#define cJSON_GetObj(object, key) (cJSON_GetObjectItem(object, key) ? cJSON_GetObjectItem(object, key) : cJSON_CreateObject())
#define cJSON_GetObj(object, key) (cJSON_GetObjectItem(object, key) ? cJSON_GetObjectItem(object, key) : NULL)

#undef EOF
#define NOP 0x00
#define SOF 0x01
#define EOF 0xFF

#define buflen  250
#define CRC_SUFFIX_LEN 5 // "#hhhh"

// Link statistics, sent as {"Link":{...}} every 5 s.
static volatile uint32_t statRxFrames, statRxCrcErr, statRxNoCrc, statRxOverflow, statTxFrames, statTxDropped;

// Frames to the Pi carry a CRC only once the Pi sent one with a valid CRC: an
// older worx_hardware can't parse the suffix (strict JSON), a newer one sends it
// with every command. Reset with the board.
static volatile bool peerCrc = false;

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), the same as worx_hardware's.
static uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

static int hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// Frames carry "#hhhh" (CRC of the text before it) in front of EOF. Checks and
// strips it in place. Returns false for a wrong CRC (frame must be dropped); a
// frame without the suffix (older worx_hardware) is accepted.
static bool frameCrcOk(char *msg) {
    size_t len = strlen(msg);
    if (len < CRC_SUFFIX_LEN || msg[len - CRC_SUFFIX_LEN] != '#') {
        statRxNoCrc++;
        return true;
    }
    uint16_t want = 0;
    for (int i = 1; i < CRC_SUFFIX_LEN; i++) {
        int v = hexval(msg[len - CRC_SUFFIX_LEN + i]);
        if (v < 0) {
            statRxNoCrc++; // not a CRC suffix after all
            return true;
        }
        want = (want << 4) | v;
    }
    msg[len - CRC_SUFFIX_LEN] = 0;
    if (crc16((const uint8_t *)msg, len - CRC_SUFFIX_LEN) != want) {
        statRxCrcErr++;
        return false;
    }
    peerCrc = true;
    return true;
}

// ---- SSP0 over GPDMA (UM10470 ch. 34) --------------------------------------
// The Pi is the SPI master and clocks 250-byte transfers almost back to back at
// 1.5 MHz. The old SSP0 interrupt moved one byte per received byte and woke at
// RX FIFO half full: ~20 us to react before the TX FIFO ran dry (the Pi got an
// extra byte) or the RX FIFO overflowed (a byte lost).
// Now two DMA channels run endlessly through linked lists that point back at
// themselves:
//  - RX (channel 1): SSP0 -> rxRing (1 KB). ROSCommsRx_Task reads the DMA write
//    position every 1 ms and takes the new bytes; 1 KB = 5.5 ms at 1.5 MHz.
//  - TX (channel 0): txHalf[0], txHalf[1] -> SSP0. When a half has been sent
//    (terminal count interrupt, one per 128 bytes) it is refilled from
//    SPI0TxStreamBuffer, the rest with NOP; the other half is being sent meanwhile,
//    so the deadline is ~0.7 ms instead of ~20 us.
// Buffers are in main SRAM: the GPDMA can't reach it only in sleep mode, which
// this firmware doesn't use (no tickless idle / WFI).

#define DMA_REQ_SSP0_TX 2   // UM10470 table 685 (DMAREQSEL bits 2/3 = 0: SSP0, not T1_MAT)
#define DMA_REQ_SSP0_RX 3

#define DMA_CTRL_SB4   (1 << 12)  // source burst 4 (SSP FIFO half)
#define DMA_CTRL_DB4   (1 << 15)  // destination burst 4
#define DMA_CTRL_SI    (1 << 26)  // source increment (widths 0 = byte)
#define DMA_CTRL_DI    (1 << 27)  // destination increment
#define DMA_CTRL_I     (1UL << 31) // terminal count interrupt

#define DMA_CFG_E          (1 << 0)
#define DMA_CFG_SRC(n)     ((n) << 1)
#define DMA_CFG_DST(n)     ((n) << 6)
#define DMA_CFG_M2P        (1 << 11)
#define DMA_CFG_P2M        (2 << 11)
#define DMA_CFG_IE         (1 << 14)
#define DMA_CFG_ITC        (1 << 15)

#define RX_RING  1024
#define RX_HALF  (RX_RING / 2)
#define TX_HALF  128

typedef struct {
    uint32_t src, dst, next, ctrl;
} DmaLli;

static uint8_t rxRing[RX_RING] __attribute__((aligned(4)));
static uint8_t txHalf[2][TX_HALF] __attribute__((aligned(4)));
static DmaLli rxLli[2] __attribute__((aligned(4)));
static DmaLli txLli[2] __attribute__((aligned(4)));
static volatile bool spiReady = false;
static uint8_t txRefill = 0; // half that finishes next (they alternate, 0 first)
static volatile uint32_t statDmaErr;

static void spiDmaInit(void) {
    ROScomms_Init(); // SSP0 pins, 8 bit, CPOL/CPHA, slave; not enabled yet

    LPC_SC->PCONP |= (1 << 29);            // PCGPDMA
    LPC_SC->DMAREQSEL &= ~((1 << DMA_REQ_SSP0_TX) | (1 << DMA_REQ_SSP0_RX));
    LPC_GPDMA->Config = 1;                  // controller enable (was missing in the 2023 attempt)
    while (!(LPC_GPDMA->Config & 1));
    LPC_GPDMACH0->CConfig = 0;
    LPC_GPDMACH1->CConfig = 0;
    LPC_GPDMA->IntTCClear = 0x03;
    LPC_GPDMA->IntErrClr = 0x03;

    memset(txHalf, NOP, sizeof(txHalf));
    uint32_t dr = (uint32_t)&LPC_SSP0->DR;
    for (int i = 0; i < 2; i++) {
        txLli[i] = (DmaLli){ (uint32_t)txHalf[i], dr, (uint32_t)&txLli[i ^ 1],
                             TX_HALF | DMA_CTRL_SB4 | DMA_CTRL_DB4 | DMA_CTRL_SI | DMA_CTRL_I };
        rxLli[i] = (DmaLli){ dr, (uint32_t)&rxRing[i * RX_HALF], (uint32_t)&rxLli[i ^ 1],
                             RX_HALF | DMA_CTRL_SB4 | DMA_CTRL_DB4 | DMA_CTRL_DI };
    }

    LPC_GPDMACH1->CSrcAddr = rxLli[0].src;
    LPC_GPDMACH1->CDestAddr = rxLli[0].dst;
    LPC_GPDMACH1->CLLI = rxLli[0].next;
    LPC_GPDMACH1->CControl = rxLli[0].ctrl;
    LPC_GPDMACH1->CConfig = DMA_CFG_E | DMA_CFG_SRC(DMA_REQ_SSP0_RX) | DMA_CFG_P2M | DMA_CFG_IE;

    LPC_GPDMACH0->CSrcAddr = txLli[0].src;
    LPC_GPDMACH0->CDestAddr = txLli[0].dst;
    LPC_GPDMACH0->CLLI = txLli[0].next;
    LPC_GPDMACH0->CControl = txLli[0].ctrl;
    LPC_GPDMACH0->CConfig = DMA_CFG_E | DMA_CFG_DST(DMA_REQ_SSP0_TX) | DMA_CFG_M2P | DMA_CFG_IE | DMA_CFG_ITC;

    NVIC_SetPriority(DMA_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(DMA_IRQn);

    LPC_SSP0->IMSC = 0;          // no SSP interrupts any more
    LPC_SSP0->DMACR = 0x03;      // RXDMAE | TXDMAE
    LPC_SSP0->CR1 |= (1 << 1);   // SSP enable, last
    spiReady = true;
}

void DMA_IRQHandler(void) {
    portBASE_TYPE woken = pdFALSE;

    if (LPC_GPDMA->IntTCStat & 0x01) {
        LPC_GPDMA->IntTCClear = 0x01;
        // Refill the half that just finished; the DMA reads the other one now. The
        // halves alternate. At the boundary the source address can still point at
        // the end of the finished half, so it only re-syncs when the DMA is
        // clearly inside the half we were about to refill (a missed interrupt).
        uint32_t src = LPC_GPDMACH0->CSrcAddr;
        uint32_t start = (uint32_t)txHalf[txRefill];
        if (src > start && src < start + TX_HALF) txRefill ^= 1;
        uint8_t *fill = txHalf[txRefill];
        txRefill ^= 1;
        size_t n = xStreamBufferReceiveFromISR(SPI0TxStreamBuffer, fill, TX_HALF, &woken);
        if (n < TX_HALF) memset(fill + n, NOP, TX_HALF - n);
    }
    uint32_t err = LPC_GPDMA->IntErrStat & 0x03;
    if (err) {
        LPC_GPDMA->IntErrClr = err;
        statDmaErr++; // bus error: the channel stops; counted in "Link"
    }
    portYIELD_FROM_ISR(woken);
}

// Moves whole frames (SOF + JSON + EOF) from RosTxQueue into the byte stream the
// SSP0 ISR clocks out. A frame goes in only when it fits completely: the old
// byte-by-byte message buffer (5 bytes of space per byte) dropped single bytes
// whenever the Pi paused the SPI clock, giving frames like "Rai:0" or "Coll".
// If the Pi stops reading, RosTxQueue fills and producers drop whole frames.
void SPI0TxQueue_Task(void *pvParameters) {
    uint8_t frame[buflen + CRC_SUFFIX_LEN + 3];
    char local_txbuf[buflen+1];

    while (1) {
        if (xQueueReceive(RosTxQueue, local_txbuf, portMAX_DELAY) == pdPASS) {
            local_txbuf[buflen] = 0;
            size_t len = strnlen(local_txbuf, buflen);
            frame[0] = SOF;
            memcpy(&frame[1], local_txbuf, len);
            // "#hhhh": CRC-16 of the text, so the Pi can drop a damaged frame
            // (a flipped digit in valid JSON would otherwise pass).
            if (peerCrc) {
                snprintf((char *)&frame[len + 1], CRC_SUFFIX_LEN + 1, "#%04X", crc16((const uint8_t *)local_txbuf, len));
                len += CRC_SUFFIX_LEN;
            }
            frame[len + 1] = EOF;
            len += 2;
            while (xStreamBufferSpacesAvailable(SPI0TxStreamBuffer) < len) {
                vTaskDelay(xDelay1);
            }
            xStreamBufferSend(SPI0TxStreamBuffer, frame, len, 0);
            statTxFrames++;
        }
    }
}

// Prints root into a frame and queues it; frees root. Returns false if the JSON
// didn't fit in buflen (never send a half-printed buffer) or the queue was full.
bool rosSendJSON(cJSON *root) {
    char local_txbuf[buflen+1];
    bool ok = cJSON_PrintPreallocated(root, local_txbuf, buflen, false);
    cJSON_Delete(root);
    if (!ok || xQueueSend(RosTxQueue, local_txbuf, xDelay10) != pdPASS) {
        statTxDropped++;
        return false;
    }
    return true;
}

// Motor commands from ROS: {"<name>": {...}}. Any valid command also counts as
// a sign of life for the SPI watchdog (was: only ping).
static const struct {
    const char *name;
    xMotorRequestType action;
} motorCommands[] = {
    { "MOTORREQ_ENABLE",   MOTORREQ_ENABLE },
    { "MOTORREQ_DISABLE",  MOTORREQ_DISABLE },
    { "MOTORREQ_SETSPEED", MOTORREQ_SETSPEED },
    { "MOTORREQ_RESETEMG", MOTORREQ_RESETEMG },
};

// One complete frame (text between SOF and EOF, 0-terminated) from the Pi.
static void handleFrame(char *local_rxbuf) {
            statRxFrames++;
            if (!frameCrcOk(local_rxbuf)) return;
            wdt_reset();
            cJSON* root = cJSON_Parse(local_rxbuf);
            if (root != NULL) {
                cJSON* command = cJSON_GetArrayItem(root, 0);
                if (command != NULL && command->string != NULL) {
                    bool known = false;
                    if (!strcasecmp(command->string, "ping")) {
                        known = true;
                        cJSON* msg = cJSON_CreateObject();
                        cJSON* obj = cJSON_CreateObject();
                        cJSON_AddItemToObject(obj, "Count", cJSON_CreateNumber(cJSON_GetInt(command, "count")));
                        cJSON_AddItemToObject(msg, "pong", obj);
                        rosSendJSON(msg);
                    } else {
                        for (size_t i = 0; i < sizeof(motorCommands) / sizeof(motorCommands[0]); i++) {
                            if (strcasecmp(command->string, motorCommands[i].name)) continue;
                            known = true;
                            xMotorMsgType MotorMsg = { .action = motorCommands[i].action, .reason = 0, .pwm = {0, 0, 0} };
                            if (MotorMsg.action == MOTORREQ_SETSPEED) {
                                MotorMsg.pwm.left = cJSON_GetInt(command, "left");
                                MotorMsg.pwm.right = cJSON_GetInt(command, "right");
                                MotorMsg.pwm.blade = cJSON_GetInt(command, "mow");
                            }
                            xQueueSend(xMotorMsgQueue, &MotorMsg, xDelay25);
                            break;
                        }
                    }
                    if (known) {
                        sensorMsg.watchdogSPI = 0;
                    } else {
                        debug("Unknown command: %s", command->string);
                    }
                }
                cJSON_Delete(root); // Will also delete children.
            }
}

// Takes the bytes the RX DMA wrote since the last call and cuts them into frames.
// Bytes outside SOF..EOF are ignored; a frame longer than the buffer is dropped
// whole.
void ROSCommsRx_Task(void *pvParameters) {
    char frame[buflen + CRC_SUFFIX_LEN + 1];
    size_t pos = 0, tail = 0;
    bool inframe = false;

    while (!spiReady) vTaskDelay(xDelay10);

    for (;;) {
        size_t head = LPC_GPDMACH1->CDestAddr - (uint32_t)rxRing; // next byte the DMA writes
        if (head >= RX_RING) head = 0; // at the wrap, before the LLI reload
        while (tail != head) {
            uint8_t b = rxRing[tail];
            tail = (tail + 1) % RX_RING;
            if (b == SOF) {
                pos = 0;
                inframe = true;
            } else if (!inframe || b == NOP) {
                // idle / padding
            } else if (b == EOF) {
                frame[pos] = 0;
                if (pos > 0) handleFrame(frame);
                inframe = false;
            } else if (pos >= sizeof(frame) - 1) {
                inframe = false;
                statRxOverflow++;
            } else {
                frame[pos++] = b;
            }
        }
        vTaskDelay(xDelay1);
    }
}

void RosCommsTx_Timer(TimerHandle_t xTimer) {
}


// Digital inputs as a bitmask, read straight from the pins (the sensor task
// copies are up to 40 ms old). Order = the Digital frame's keys.
static uint8_t digitalInputs(void) {
    return  (GPIO_CHK_PIN(SENSOR_STUCK)     << 0) | (GPIO_CHK_PIN(SENSOR_STUCK2)    << 1) |
            (GPIO_CHK_PIN(SENSOR_DOOR)      << 2) | (GPIO_CHK_PIN(SENSOR_DOOR2)     << 3) |
            (GPIO_CHK_PIN(SENSOR_LIFT)      << 4) | (GPIO_CHK_PIN(SENSOR_COLLISION) << 5) |
            (GPIO_CHK_PIN(SENSOR_STOP)      << 6) | (sensorMsg.rain                 << 7);
}

static void sendDigital(uint8_t in) {
    static const char * const names[] = {"Stuck", "Stuck2", "Door", "Door2", "Lift", "Collision", "Stop", "Rain"};
    cJSON* root = cJSON_CreateObject();
    cJSON* obj = cJSON_CreateObject();
    for (int i = 0; i < 8; i++) cJSON_AddItemToObject(obj, names[i], cJSON_CreateNumber((in >> i) & 1));
    cJSON_AddItemToObject(root, "Digital", obj);
    rosSendJSON(root);
}

void ROSCommsTx_Task(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(50);

    vTaskDelay(xDelay1000);
    wdt_reset();
    spiDmaInit();
    

    


    xBoundaryMsgType BoundaryMsg;
    HeapStats_t xHeapStats;

    char local_txbuf[buflen+1];
    
    uint16_t counter = 0;
    uint8_t printmsg = 0;
    uint8_t lastDigital = digitalInputs();

    debug("ROSComms started...");
    //debug("CoreClk: %i Perclk: %i PCLKDIV: %i", SystemCoreClock, PeripheralClock, (LPC_SC->PCLKSEL & 0x1f));

    // Check if reset was caused by WDT
    if(LPC_SC->RSID & (1 << 2)) {
        debug("!!!! Reset was caused by watchdog timer !!!!");
        // Clear the watchdog reset flag
        LPC_SC->RSID |= (1 << 2);
    }

    xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        xJSONMessageType JSONMsg;
        if (xQueueReceive(xJSONMessageQueue, &JSONMsg, 0) == pdPASS) {
            cJSON* root = cJSON_CreateObject();
            cJSON_AddStringToObject(root, JSONMsg.topic, JSONMsg.value);
            rosSendJSON(root);
        }
        /*
        // IMU Data:
        cJSON* root = cJSON_CreateObject();
        cJSON* obj = cJSON_CreateObject();
        cJSON_AddItemToObject(obj, "Yaw", cJSON_CreateNumber(round(sensorMsg.gyroYaw * 10000)));
        cJSON_AddItemToObject(obj, "Pitch", cJSON_CreateNumber(round(sensorMsg.gyroPitch * 10000)));
        cJSON_AddItemToObject(obj, "Roll", cJSON_CreateNumber(round(sensorMsg.gyroRoll * 10000)));
        cJSON_AddItemToObject(obj, "AccX", cJSON_CreateNumber(round(sensorMsg.accelX)));
        cJSON_AddItemToObject(obj, "AccY", cJSON_CreateNumber(round(sensorMsg.accelY)));
        cJSON_AddItemToObject(obj, "AccZ", cJSON_CreateNumber(round(sensorMsg.accelZ)));
        cJSON_AddItemToObject(root, "I2C_IMU", obj);
        rosSendJSON(root);
        */
        // Digital at once on any change (plus every 1.25 s in the rotation below)
        uint8_t digital = digitalInputs();
        if (digital != lastDigital) {
            lastDigital = digital;
            sendDigital(digital);
        }

        cJSON* root = cJSON_CreateObject();
        cJSON* obj = cJSON_CreateObject();
        cJSON_AddItemToObject(obj, "Left", cJSON_CreateNumber(sensorMsg.motorPulseLeft));
        cJSON_AddItemToObject(obj, "Right", cJSON_CreateNumber(sensorMsg.motorPulseRight));
        cJSON_AddItemToObject(obj, "Mow", cJSON_CreateNumber(sensorMsg.motorPulseBlade));
        cJSON_AddItemToObject(obj, "DirLeft", cJSON_CreateNumber(!GPIO_CHK_PIN(MOTOR_LEFT_FORWARD)));
        cJSON_AddItemToObject(obj, "DirRight", cJSON_CreateNumber(GPIO_CHK_PIN(MOTOR_RIGHT_FORWARD)));
        cJSON_AddItemToObject(obj, "Emergancy", cJSON_CreateNumber(sensorMsg.emergancyStop));
        cJSON_AddItemToObject(obj, "EmgReason", cJSON_CreateNumber(sensorMsg.emergencyReason));
        cJSON_AddItemToObject(obj, "Motors", cJSON_CreateNumber(sensorMsg.motorsEnabled));
        cJSON_AddItemToObject(obj, "BlockForward", cJSON_CreateNumber(sensorMsg.blockForward));
        cJSON_AddItemToObject(obj, "Bumps", cJSON_CreateNumber(sensorMsg.bumps));
        cJSON_AddItemToObject(obj, "BladeLock", cJSON_CreateNumber(sensorMsg.bladeLockout));
        cJSON_AddItemToObject(obj, "ms", cJSON_CreateNumber(sensorMsg.globalticksms)); // board time of this sample
        cJSON_AddItemToObject(root, "MotorPulse", obj);
        rosSendJSON(root);

        // Boundary wire data:
		if (xQueueReceive(xBoundaryMsgQueue, &BoundaryMsg, 0) == pdPASS) {
            root = cJSON_CreateObject();
            obj = cJSON_CreateObject();
            cJSON_AddItemToObject(obj, "sleft", cJSON_CreateNumber(atoi(BoundaryMsg.sleft)));
            cJSON_AddItemToObject(obj, "sright", cJSON_CreateNumber(atoi(BoundaryMsg.sright)));
            cJSON_AddItemToObject(obj, "nleft", cJSON_CreateNumber(atoi(BoundaryMsg.nleft)));
            cJSON_AddItemToObject(obj, "nright", cJSON_CreateNumber(atoi(BoundaryMsg.nright)));
            cJSON_AddItemToObject(root, "Boundary", obj);    
            rosSendJSON(root);
        }
if (!(counter % 5)) {
        root = cJSON_CreateObject();
        obj = cJSON_CreateObject();
        switch (printmsg) {
            case 1:
                cJSON_AddItemToObject(obj, "mV", cJSON_CreateNumber(sensorMsg.batteryVolt));
                cJSON_AddItemToObject(obj, "mA", cJSON_CreateNumber(sensorMsg.batteryChargeCurrent));
                cJSON_AddItemToObject(obj, "Temp", cJSON_CreateNumber(sensorMsg.batteryTemp));
                cJSON_AddItemToObject(obj, "CellLow", cJSON_CreateNumber(sensorMsg.batteryCellLow));
                cJSON_AddItemToObject(obj, "CellHigh", cJSON_CreateNumber(sensorMsg.batteryCellHigh));
                cJSON_AddItemToObject(obj, "InCharger", cJSON_CreateNumber(sensorMsg.inCharger));
                cJSON_AddItemToObject(obj, "State", cJSON_CreateString(powerStateStrings[sensorMsg.powerState]));
                cJSON_AddItemToObject(obj, "Contact", cJSON_CreateNumber(GPIO_CHK_PIN(CHARGER_CONNECTED)));
                cJSON_AddItemToObject(obj, "Enable", cJSON_CreateNumber(GPIO_CHK_PIN(CHARGER_ENABLE)));
                cJSON_AddItemToObject(root, "Battery", obj);
                break;
            case 2:
                cJSON_Delete(obj);
                cJSON_Delete(root);
                root = NULL;
                sendDigital(lastDigital = digitalInputs());
                break;
            case 3:
                cJSON_AddItemToObject(obj, "Rain", cJSON_CreateNumber(sensorMsg.rainAnalog));
                cJSON_AddItemToObject(obj, "boardTemp", cJSON_CreateNumber(sensorMsg.boardTemp)); // RAW
                cJSON_AddItemToObject(root, "Analog", obj);
                break;
            case 4:
                cJSON_AddItemToObject(obj, "Left", cJSON_CreateNumber(LPC_PWM1->MR4));
                cJSON_AddItemToObject(obj, "Right", cJSON_CreateNumber(LPC_PWM1->MR5));
                cJSON_AddItemToObject(obj, "Mow", cJSON_CreateNumber(LPC_PWM1->MR1));
                cJSON_AddItemToObject(root, "MotorPWM", obj);
                break;
            /*case 5:
                cJSON_AddItemToObject(obj, "Left", cJSON_CreateNumber(sensorMsg.motorpulseleft));
                cJSON_AddItemToObject(obj, "Right", cJSON_CreateNumber(sensorMsg.motorpulseright));
                cJSON_AddItemToObject(obj, "Mow", cJSON_CreateNumber(sensorMsg.motorpulseblade));
                cJSON_AddItemToObject(root, "MotorPulse", obj);
                break;*/
            case 5:
                cJSON_AddItemToObject(obj, "Left", cJSON_CreateNumber(sensorMsg.motorCurrentLeft));
                cJSON_AddItemToObject(obj, "Right", cJSON_CreateNumber(sensorMsg.motorCurrentRight));
                cJSON_AddItemToObject(obj, "Mow", cJSON_CreateNumber(sensorMsg.motorCurrentBlade));

                cJSON_AddItemToObject(root, "MotorCurrent", obj);
                printmsg = 0;
                break;
            default:
                printmsg = 0;
                break;
            /*case 7:
                cJSON_AddItemToObject(obj, "QueueWait", cJSON_CreateNumber(uxQueueMessagesWaiting(RosTxQueue)));
                cJSON_AddItemToObject(obj, "TxFree", cJSON_CreateNumber(xMessageBufferSpacesAvailable(SPI0TxMessageBuffer)));
                cJSON_AddItemToObject(obj, "FreeH", cJSON_CreateNumber(xPortGetFreeHeapSize()));
                cJSON_AddItemToObject(root, "DEBUG", obj);
                
                break;
            case 8:
                cJSON_AddItemToObject(obj, "AccelX", cJSON_CreateNumber(sensorMsg.AccelX));
                cJSON_AddItemToObject(obj, "AccelY", cJSON_CreateNumber(sensorMsg.AccelY));
                cJSON_AddItemToObject(obj, "AccelZ", cJSON_CreateNumber(sensorMsg.AccelZ));
                cJSON_AddItemToObject(root, "I2C_MMA8452Q", obj);
                break;
            case 9:
                int spibus = SystemCoreClock / (LPC_SC->PCLKSEL * (LPC_SSP0->CPSR * (1 + (LPC_SSP0->CR0 >> 8))));
                cJSON_AddItemToObject(obj, "CpuClk", cJSON_CreateNumber(SystemCoreClock));
                cJSON_AddItemToObject(obj, "SpiClk", cJSON_CreateNumber(spibus));
                cJSON_AddItemToObject(root, "Debug", obj);
                printmsg = 0;
                break;*/
        }
        printmsg++;

        if (root) rosSendJSON(root);
}
        
        if (!(counter % 100)) { // 5 s
            root = cJSON_CreateObject();
            obj = cJSON_CreateObject();
            cJSON_AddItemToObject(obj, "RxFrames", cJSON_CreateNumber(statRxFrames));
            cJSON_AddItemToObject(obj, "RxCrcErr", cJSON_CreateNumber(statRxCrcErr));
            cJSON_AddItemToObject(obj, "RxNoCrc", cJSON_CreateNumber(statRxNoCrc));
            cJSON_AddItemToObject(obj, "RxOverflow", cJSON_CreateNumber(statRxOverflow));
            cJSON_AddItemToObject(obj, "TxFrames", cJSON_CreateNumber(statTxFrames));
            cJSON_AddItemToObject(obj, "TxDropped", cJSON_CreateNumber(statTxDropped));
            cJSON_AddItemToObject(obj, "DmaErr", cJSON_CreateNumber(statDmaErr));
            cJSON_AddItemToObject(root, "Link", obj);
            rosSendJSON(root);
        }

        if (!(counter % 1000)) {
            for ( int i = 0; i < taskcounter; i++) {
                sprintf(local_txbuf,"Task %i:%s StackHigh: %li ",i , pcTaskGetName(xHandle[i]), uxTaskGetStackHighWaterMark(xHandle[i]));
                xQueueSend(RosTxQueue, local_txbuf, xDelay10);
            }
            vPortGetHeapStats( &xHeapStats );
            sprintf(local_txbuf, "HeapStats: Free:%i (min: %i) Allocs: %i Frees: %i", xHeapStats.xAvailableHeapSpaceInBytes, 
                xHeapStats.xMinimumEverFreeBytesRemaining, xHeapStats.xNumberOfSuccessfulAllocations, xHeapStats.xNumberOfSuccessfulFrees); 
            xQueueSend(RosTxQueue, local_txbuf, xDelay10);
        }
        
        counter++;
        int stack = uxTaskGetStackHighWaterMark(NULL);
        if (stack < 50) debug("Task ROSComms_Task has %u words left in stack.", stack);
    }
}

void ROSCommsTest_Task(void *pvParameters) {
}

void debug( const char* format, ... ) {
    char buf[buflen+1];
    va_list args;
    int len = snprintf( buf, sizeof(buf), "DEBUG: " );
    va_start( args, format );
    vsnprintf( buf + len, sizeof(buf) - len, format, args );
    va_end( args );
    xQueueSend(RosTxQueue, buf, xDelay10);
}

#pragma import(__use_no_semihosting_swi)

__attribute__((used)) int _write(int fd, char *ptr, int len) {
    int i = 0;
    /*char ch;
    while (*ptr && (i < len)) {
        ch = *ptr;
        xxxxx -- xMessageBufferSend(SPI0TxMessageBuffer, (char*)&ch, 1, xDelay1);
        NOT TASK/THREAD SAFE! Crashes! Fill a buffer and xQueueSend to RosTxQueue
        i++;
        ptr++;
    }*/

    return i;
}