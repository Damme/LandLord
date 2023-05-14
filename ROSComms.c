#include "ROSComms.h"
#include "global.h"
#include "common.h"
#include "queue.h"
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "task.h"
#include "timers.h"
#include "cJSON.h"
#include <stdarg.h>

#define cJSON_GetInt(object, key) (cJSON_GetObjectItem(object, key) ? cJSON_GetObjectItem(object, key)->valueint : 0)
#define cJSON_GetStr(object, key) (cJSON_GetObjectItem(object, key) ? cJSON_GetObjectItem(object, key)->valuestring : "")
//#define cJSON_GetObj(object, key) (cJSON_GetObjectItem(object, key) ? cJSON_GetObjectItem(object, key) : cJSON_CreateObject())
#define cJSON_GetObj(object, key) (cJSON_GetObjectItem(object, key) ? cJSON_GetObjectItem(object, key) : NULL)

#define NOP 0x00
#define SOF 0x01
#define EOF 0xFF

#define buflen  250

uint8_t SSP0_rxpos = 0;
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
        if (xMessageBufferReceiveFromISR(SPI0TxMessageBuffer, &txByte, 1, &xHigherPriorityTaskWoken) == pdFAIL) {
            txByte=NOP;
        }
        LPC_SSP0->DR = txByte;

        // Process the received byte
        if (rxByte == SOF) { // If start of frame
            // Reset the rxpos index to 0
            SSP0_rxpos = 0;
        } else if (rxByte == EOF || (SSP0_rxpos >= buflen - 1)) {
            SSP0_rxbuf[SSP0_rxpos++] = 0;
            // Send received data to the RxMessageBuffer
            if (SSP0_rxpos > 0) xMessageBufferSendFromISR(SPI0RxMessageBuffer, &SSP0_rxbuf, SSP0_rxpos, &xHigherPriorityTaskWoken);
            SSP0_rxpos = 0;
        } else if (rxByte != NOP) { // If regular data byte
            SSP0_rxbuf[SSP0_rxpos++] = rxByte; // Add received byte to the rxbuf
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

void SPI0TxQueue_Task(void *pvParameters) {
    char local_txbuf[buflen+1];
    char txByte;
    size_t txpos;

    while (1) {
        // Check if the SPI0TxMessageBuffer has room for message
        if (xMessageBufferSpaceAvailable(SPI0TxMessageBuffer) > buflen + 2) {
            // Receive data from the TxMessageBuffer
            if (xQueueReceive(RosTxQueue, local_txbuf, xDelay1) > 0) {
                // Enqueue the SOF byte
                txByte = SOF;
                xMessageBufferSend(SPI0TxMessageBuffer, &txByte, 1, xDelay1);

                // Enqueue the received data
                for (txpos = 0; txpos < strnlen(local_txbuf, buflen); txpos++) {
                    xMessageBufferSend(SPI0TxMessageBuffer, &local_txbuf[txpos], 1, xDelay1);
                }

                // Enqueue the EOF byte
                txByte = EOF;
                xMessageBufferSend(SPI0TxMessageBuffer, &txByte, 1, xDelay1);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void ROSCommsRx_Task(void *pvParameters) {
    char local_rxbuf[buflen+1];

    for (;;) {
        while (xMessageBufferReceive(SPI0RxMessageBuffer, &local_rxbuf, buflen, xDelay10)) {
            cJSON* root = cJSON_Parse(local_rxbuf);
            if (root != NULL) {
                cJSON* command = cJSON_GetArrayItem(root, 0);
                if (command != NULL) {
                   if (!strcasecmp(command->string, "ping")) {
                        cJSON* REQ = cJSON_GetObj(root, "ping");
                        if (REQ != NULL) {
                            /*xJSONMessageType JSONMsg = {"pong", {0}};
                            strcpy(JSONMsg.value, cJSON_CreateNumber(cJSON_GetInt(REQ, "count")));
                            xQueueSend(xJSONMessageQueue, &JSONMsg, xDelay25);*/

                            //debug("Pong! last: %i", watchdogSPI);
                            char local_txbuf[buflen+1];
                            cJSON* msg = cJSON_CreateObject();
                            cJSON* obj = cJSON_CreateObject();
                            cJSON_AddItemToObject(obj, "Count", cJSON_CreateNumber(cJSON_GetInt(REQ, "count")));
                            cJSON_AddItemToObject(msg, "pong", obj);
                            cJSON_PrintPreallocated(msg, local_txbuf, buflen, false);
                            xQueueSend(RosTxQueue, local_txbuf, xDelay25);
                            cJSON_Delete(msg);
                            sensorMsg.watchdogSPI=0;
                        }
                    } else if (!strcasecmp(command->string, "MOTORREQ_DISABLE")) {
                        cJSON* REQ = cJSON_GetObj(root, "MOTORREQ_DISABLE");
                        if (REQ != NULL) {
                            xMotorMsgType MotorMsg;
                            MotorMsg.action = MOTORREQ_DISABLE;
                            MotorMsg.pwm.left = 0;
                            MotorMsg.pwm.right = 0;
                            MotorMsg.pwm.blade = 0;
                            xQueueSend(xMotorMsgQueue, &MotorMsg, xDelay25);
                        }
                    } else if (!strcasecmp(command->string, "MOTORREQ_ENABLE")) {
                        cJSON* REQ = cJSON_GetObj(root, "MOTORREQ_ENABLE");
                        if (REQ != NULL) {
                            xMotorMsgType MotorMsg;
                            MotorMsg.action = MOTORREQ_ENABLE;
                            MotorMsg.pwm.left = 0;
                            MotorMsg.pwm.right = 0;
                            MotorMsg.pwm.blade = 0;
                            xQueueSend(xMotorMsgQueue, &MotorMsg, xDelay25);
                        }
                    } else if (!strcasecmp(command->string, "MOTORREQ_SETSPEED")) {
                        cJSON* REQ = cJSON_GetObj(root, "MOTORREQ_SETSPEED");
                        if (REQ != NULL) {
                            xMotorMsgType MotorMsg;
                            MotorMsg.action = MOTORREQ_SETSPEED;
                            MotorMsg.pwm.left = cJSON_GetInt(REQ, "left");
                            MotorMsg.pwm.right = cJSON_GetInt(REQ, "right");
                            MotorMsg.pwm.blade = cJSON_GetInt(REQ, "mow");
                            //debug("setpwm: l%d r%d m%d\n", MotorMsg.pwm.left, MotorMsg.pwm.right, MotorMsg.pwm.blade);
                            xQueueSend(xMotorMsgQueue, &MotorMsg, xDelay25);
                        }
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

void ROSCommsTx_Task(void *pvParameters) {
    vTaskDelay(xDelay1000);
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

    debug("ROSComms started...");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(20));
        xJSONMessageType JSONMsg;
        if (xQueueReceive(xJSONMessageQueue, &JSONMsg, 0) == pdPASS) {
            cJSON* root = cJSON_CreateObject();
            cJSON_AddStringToObject(root, JSONMsg.topic, JSONMsg.value);
            cJSON_PrintPreallocated(root, local_txbuf, buflen, false);
            xQueueSend(RosTxQueue, local_txbuf, xDelay10);
            cJSON_Delete(root);
        }
        
        // IMU Data:
        cJSON* root = cJSON_CreateObject();
        cJSON* obj = cJSON_CreateObject();
        cJSON_AddItemToObject(obj, "Yaw", cJSON_CreateNumber(sensorMsg.gyroYaw));
        cJSON_AddItemToObject(obj, "Pitch", cJSON_CreateNumber(sensorMsg.gyroPitch));
        cJSON_AddItemToObject(obj, "Roll", cJSON_CreateNumber(sensorMsg.gyroRoll));
        cJSON_AddItemToObject(obj, "AccX", cJSON_CreateNumber(sensorMsg.accelX));
        cJSON_AddItemToObject(obj, "AccY", cJSON_CreateNumber(sensorMsg.accelY));
        cJSON_AddItemToObject(obj, "AccZ", cJSON_CreateNumber(sensorMsg.accelZ));
        cJSON_AddItemToObject(root, "I2C_IMU", obj);
        cJSON_PrintPreallocated(root, local_txbuf, buflen, false);
        xQueueSend(RosTxQueue, local_txbuf, xDelay10);
        cJSON_Delete(root);

        root = cJSON_CreateObject();
        obj = cJSON_CreateObject();
        cJSON_AddItemToObject(obj, "Left", cJSON_CreateNumber(sensorMsg.motorPulseLeft));
        cJSON_AddItemToObject(obj, "Right", cJSON_CreateNumber(sensorMsg.motorPulseRight));
        cJSON_AddItemToObject(obj, "Mow", cJSON_CreateNumber(sensorMsg.motorPulseBlade));
        cJSON_AddItemToObject(obj, "DirLeft", cJSON_CreateNumber(!GPIO_CHK_PIN(MOTOR_LEFT_FORWARD)));
        cJSON_AddItemToObject(obj, "DirRight", cJSON_CreateNumber(GPIO_CHK_PIN(MOTOR_RIGHT_FORWARD)));
        cJSON_AddItemToObject(obj, "Emergancy", cJSON_CreateNumber(sensorMsg.emergancyStop));
        cJSON_AddItemToObject(obj, "BlockForward", cJSON_CreateNumber(sensorMsg.blockForward));
        cJSON_AddItemToObject(root, "MotorPulse", obj);
        cJSON_PrintPreallocated(root, local_txbuf, buflen, false);
        xQueueSend(RosTxQueue, local_txbuf, xDelay10);
        cJSON_Delete(root);

        // Boundary wire data:
		if (xQueueReceive(xBoundaryMsgQueue, &BoundaryMsg, 0) == pdPASS) {
            root = cJSON_CreateObject();
            obj = cJSON_CreateObject();
            cJSON_AddItemToObject(obj, "sleft", cJSON_CreateNumber(atoi(BoundaryMsg.sleft)));
            cJSON_AddItemToObject(obj, "sright", cJSON_CreateNumber(atoi(BoundaryMsg.sright)));
            cJSON_AddItemToObject(obj, "nleft", cJSON_CreateNumber(atoi(BoundaryMsg.nleft)));
            cJSON_AddItemToObject(obj, "nright", cJSON_CreateNumber(atoi(BoundaryMsg.nright)));
            cJSON_AddItemToObject(root, "Boundary", obj);    
            cJSON_PrintPreallocated(root, local_txbuf, buflen, false);
            xQueueSend(RosTxQueue, local_txbuf, xDelay10);
            cJSON_Delete(root);
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
                cJSON_AddItemToObject(root, "Battery", obj);
                break;
            case 2:
                cJSON_AddItemToObject(obj, "Stuck", cJSON_CreateNumber(sensorMsg.stuck));
                cJSON_AddItemToObject(obj, "Stuck2", cJSON_CreateNumber(sensorMsg.stuck2));
                cJSON_AddItemToObject(obj, "Door", cJSON_CreateNumber(sensorMsg.door));
                cJSON_AddItemToObject(obj, "Door2", cJSON_CreateNumber(sensorMsg.door2));
                cJSON_AddItemToObject(obj, "Lift", cJSON_CreateNumber(sensorMsg.lift));
                cJSON_AddItemToObject(obj, "Collision", cJSON_CreateNumber(sensorMsg.collision));
                cJSON_AddItemToObject(obj, "Stop", cJSON_CreateNumber(sensorMsg.stop));
                cJSON_AddItemToObject(obj, "Rain", cJSON_CreateNumber(sensorMsg.rain));
                cJSON_AddItemToObject(root, "Digital", obj);
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
                cJSON_AddItemToObject(obj, "MowRPM", cJSON_CreateNumber(sensorMsg.motorCurrentBlade));
                cJSON_AddItemToObject(root, "MotorCurrent", obj);
                printmsg = 0;
                break;
            default:
                printmsg = 0;
                break;
            /*case 7:
                cJSON_AddItemToObject(obj, "Yaw", cJSON_CreateNumber(sensorMsg.GyroYaw));
                cJSON_AddItemToObject(obj, "Pitch", cJSON_CreateNumber(sensorMsg.GyroPitch));
                cJSON_AddItemToObject(obj, "Roll", cJSON_CreateNumber(sensorMsg.GyroRoll));
                cJSON_AddItemToObject(obj, "AccX", cJSON_CreateNumber(sensorMsg.AccelX));
                cJSON_AddItemToObject(obj, "AccY", cJSON_CreateNumber(sensorMsg.AccelY));
                cJSON_AddItemToObject(obj, "AccZ", cJSON_CreateNumber(sensorMsg.AccelZ));
                cJSON_AddItemToObject(root, "I2C_IMU", obj);
                
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

        cJSON_PrintPreallocated(root, local_txbuf, buflen, false);
        xQueueSend(RosTxQueue, local_txbuf, xDelay10);
        cJSON_Delete(root);
}
        
        if (!(counter % 1000)) {
            int len = 0;
            for ( int i = 0; i < taskcounter; i++) {
                len = sprintf(local_txbuf,"Task %i:%s StackHigh: %li ",i , pcTaskGetName(xHandle[i]), uxTaskGetStackHighWaterMark(xHandle[i]));
                xQueueSend(RosTxQueue, local_txbuf, xDelay10);
            }
            vPortGetHeapStats( &xHeapStats );
            len = sprintf(local_txbuf, "HeapStats: Free:%i (min: %i) Allocs: %i Frees: %i", xHeapStats.xAvailableHeapSpaceInBytes, 
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
    uint8_t len;
    va_list args;
    len = sprintf( buf, "DEBUG: " );
    va_start( args, format );
    len += vsprintf( buf + len, format, args );
    va_end( args );
    //len += sprintf( buf + len, "\n" );
    buf[len++]=0;
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