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

uint8_t SSP0_rxpos = 0;
bool SSP0_inframe = false;
uint8_t SSP0_rxbuf[buflen+1];

void SSP0_IRQHandler(void) { // SSP0_IRQn 14 (lpc1788)
    // Disable interrupts and clear the interrupt mask
    portDISABLE_INTERRUPTS();
    LPC_SSP0->IMSC &= ~(1 << 2);

    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    uint8_t rxByte; // Received byte from SPI
    uint8_t txByte = NOP; // Byte to transmit via SPI

    // Process received data while Receive FIFO is not empty
    while (LPC_SSP0->SR & (1 << 2)) {
        rxByte = LPC_SSP0->DR; // Read the received byte

        // Transmit the byte
        if (xStreamBufferReceiveFromISR(SPI0TxStreamBuffer, &txByte, 1, &xHigherPriorityTaskWoken) == 0) {
            txByte=NOP;
        }
        LPC_SSP0->DR = txByte;

        // Process the received byte. Bytes outside SOF..EOF are ignored, and a
        // frame that overflows the buffer is dropped whole (not passed on cut).
        if (rxByte == SOF) {
            SSP0_rxpos = 0;
            SSP0_inframe = true;
        } else if (!SSP0_inframe || rxByte == NOP) {
            // idle / padding
        } else if (rxByte == EOF) {
            SSP0_rxbuf[SSP0_rxpos++] = 0;
            if (SSP0_rxpos > 1) xMessageBufferSendFromISR(SPI0RxMessageBuffer, &SSP0_rxbuf, SSP0_rxpos, &xHigherPriorityTaskWoken);
            SSP0_rxpos = 0;
            SSP0_inframe = false;
        } else if (SSP0_rxpos >= buflen - 1) {
            SSP0_rxpos = 0;
            SSP0_inframe = false;
        } else {
            SSP0_rxbuf[SSP0_rxpos++] = rxByte;
        }
        
        // Wait for the transmission to complete
        // Do we really need to wait?
        //while (!(LPC_SSP0->SR & (1 << 4)));
    }

    // Re-enable the interrupt mask and enable interrupts
    LPC_SSP0->IMSC |= (1 << 2);
    portENABLE_INTERRUPTS();

    // Yield from ISR if a higher priority task was woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Moves whole frames (SOF + JSON + EOF) from RosTxQueue into the byte stream the
// SSP0 ISR clocks out. A frame goes in only when it fits completely: the old
// byte-by-byte message buffer (5 bytes of space per byte) dropped single bytes
// whenever the Pi paused the SPI clock, giving frames like "Rai:0" or "Coll".
// If the Pi stops reading, RosTxQueue fills and producers drop whole frames.
void SPI0TxQueue_Task(void *pvParameters) {
    uint8_t frame[buflen+2];
    char local_txbuf[buflen+1];

    while (1) {
        if (xQueueReceive(RosTxQueue, local_txbuf, portMAX_DELAY) == pdPASS) {
            local_txbuf[buflen] = 0;
            size_t len = strnlen(local_txbuf, buflen);
            frame[0] = SOF;
            memcpy(&frame[1], local_txbuf, len);
            frame[len + 1] = EOF;
            len += 2;
            while (xStreamBufferSpacesAvailable(SPI0TxStreamBuffer) < len) {
                vTaskDelay(xDelay1);
            }
            xStreamBufferSend(SPI0TxStreamBuffer, frame, len, 0);
        }
    }
}

// Prints root into a frame and queues it; frees root. Returns false if the JSON
// didn't fit in buflen (never send a half-printed buffer) or the queue was full.
bool rosSendJSON(cJSON *root) {
    char local_txbuf[buflen+1];
    bool ok = cJSON_PrintPreallocated(root, local_txbuf, buflen, false);
    cJSON_Delete(root);
    if (!ok) return false;
    return xQueueSend(RosTxQueue, local_txbuf, xDelay10) == pdPASS;
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

void ROSCommsRx_Task(void *pvParameters) {
    char local_rxbuf[buflen+1];

    for (;;) {
        while (xMessageBufferReceive(SPI0RxMessageBuffer, &local_rxbuf, buflen, xDelay10)) {
            local_rxbuf[buflen] = 0;
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
            memset(local_rxbuf, 0, buflen);
        }
        vTaskDelay(xDelay10);
    }
}

void RosCommsTx_Timer(TimerHandle_t xTimer) {
}

/*
void DMA_IRQHandler(void) {
    // Check if DMA channel 0 (TX) has completed the transfer
    if (LPC_GPDMA->IntTCStat & (1 << 0)) {
        // Clear the interrupt
        LPC_GPDMA->IntTCClear = 1 << 0;

        // Handle the completion of the DMA transfer (if needed)
    }
}


void spi0_Transmit_DMA(uint8_t *data, size_t length) {
    // Configure DMA channel 0 for SSP0 TX
    LPC_GPDMACH0->CSrcAddr = (uint32_t)data;
    LPC_GPDMACH0->CDestAddr = (uint32_t)&LPC_SSP0->DR;
    LPC_GPDMACH0->CControl = (length << 0) | (1 << 18) | (1 << 21) | (1 << 31);  // Transfer length, SSP0 TX, Memory-to-Peripheral
    LPC_GPDMACH0->CConfig = (1 << 0) | (1 << 1) | (10 << 6) | (1 << 11);  // Enable channel, SSP0 TX
    LPC_GPDMACH0->CConfig |= (1 << 0);

    // Clear DMA interrupts
    //LPC_GPDMA->IntTCClear = 1 << 0;  // Clear DMA channel 0 interrupt
}
*/

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
    ROScomms_Init();
    

    


    xBoundaryMsgType BoundaryMsg;
    HeapStats_t xHeapStats;

    NVIC_SetPriority(SSP0_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    LPC_SSP0->IMSC |= (1 << 2); // 1 RTIM 2 RXIM Rx FIFO is at least half full.  3 TXIM Tx FIFO is at least half empty.
    // Software should write the appropriate control information to the other SSP registers and interrupt controller registers, before setting this bit.
    LPC_SSP0->CR1 |= (1 << 1); // SSP Enable.
    NVIC_EnableIRQ(SSP0_IRQn);
    char local_txbuf[buflen+1];
    
    uint16_t counter = 0;
    uint8_t printmsg = 0;
    uint8_t lastDigital = digitalInputs();

    debug("ROSComms started...");
    //debug("CoreClk: %i Perclk: %i PCLKDIV: %i", SystemCoreClock, PeripheralClock, (LPC_SC->PCLKSEL & 0x1f));
/*
    // Enable GPDMA power
    LPC_SC->PCONP |= (1 << 29);

    // Enable GPDMA controller
    LPC_SSP0->DMACR = 0x02;

    // Clear DMA interrupts
    //LPC_GPDMA->IntTCClear = 0xFF;
    //LPC_GPDMA->IntErrClr = 0xFF;

    uint8_t test_message[] = "xHello, DMA SPI!x";
//    test_message[0]=0x01;
//    test_message[17]=0xff;
//    test_message[18]=0x00;
    spi0_Transmit_DMA(test_message, sizeof(test_message));
    while (!(LPC_GPDMACH0->CConfig & (1 << 17))) {
        // Wait until terminal count request (DMA transfer complete)
    }
    LPC_GPDMA->IntTCClear = (1 << 0);
    */

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
                cJSON_AddItemToObject(obj, "RxFree", cJSON_CreateNumber(xMessageBufferSpacesAvailable(SPI0RxMessageBuffer)));
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