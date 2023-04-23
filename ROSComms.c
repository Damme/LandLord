#include "ROSComms.h"
#include "global.h"
#include "common.h"
#include "queue.h"
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "task.h"
#include "cJSON.h"
#include <stdarg.h>

#define cJSON_GetInt(object, key) (cJSON_GetObjectItem(object, key) ? cJSON_GetObjectItem(object, key)->valueint : 0)
#define cJSON_GetStr(object, key) (cJSON_GetObjectItem(object, key) ? cJSON_GetObjectItem(object, key)->valuestring : "")
#define cJSON_GetObj(object, key) (cJSON_GetObjectItem(object, key) ? cJSON_GetObjectItem(object, key) : cJSON_CreateObject())


//xQueueHandle SPI0RxQueue;

MessageBufferHandle_t RxMessageBuffer;


#define NOP 0x00
#define SOF 0x01
#define EOF 0xFF

#define buflen  250

volatile uint8_t frame = 0;
volatile uint8_t oldframe = 0;
volatile uint8_t rxpos = 0;
volatile uint8_t rxbuf[buflen];
/*
void SSP0_IRQHandler(void) { // SSP0_IRQn 14 (lpc1788)
    portDISABLE_INTERRUPTS();
    LPC_SSP0->IMSC &= ~(1 << 2);

    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    uint8_t rxByte;
    uint8_t txByte=0x00;
    char local_txbuf[buflen];    

    while ( LPC_SSP0->SR & (1<<2) ) { // while Receive FIFO Not Empty
        rxByte = LPC_SSP0->DR; 

        if (rxByte == 0xff) { // if start of frame
            oldframe=frame;
            frame++;
            rxbuf[rxpos++] = 0;
            if (rxpos > 0) xMessageBufferSendFromISR(RxMessageBuffer, &rxbuf, rxpos, NULL);
            rxpos = 0;
            // Recieve from message buffer and put in byte buffer
            // This is slow ( too many calls to xQueueSendFromISR )
            // maybe replace SPI0TxQueue with message buffer?
            if (xMessageBufferReceiveFromISR(TxMessageBuffer, &local_txbuf, sizeof(local_txbuf), &xHigherPriorityTaskWoken)) {
                for (int i = 0 ; i < strlen(local_txbuf) ; i++) {
                    xQueueSendFromISR(SPI0TxQueue, &local_txbuf[i], &xHigherPriorityTaskWoken);
                }
            }
        } else {
            if (oldframe != frame && (xQueueReceiveFromISR(SPI0TxQueue, &txByte, &xHigherPriorityTaskWoken) == pdFAIL) ) {
                oldframe=frame;
                txByte=0;
            }

            if (rxByte != 0x00 && rxByte != 0xff) {
                rxbuf[rxpos++] = rxByte;
            }
        }
        LPC_SSP0->DR = txByte;
        while (!(LPC_SSP0->SR & (1<<4)));
  
    }

    LPC_SSP0->IMSC |= (1 << 2);
    portENABLE_INTERRUPTS();
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
*/

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

        // Process the received byte
        if (rxByte == SOF) { // If start of frame
            // Reset the rxpos index to 0
            rxpos = 0;
        } else if (rxByte == EOF) {
            rxbuf[rxpos++] = 0;
            // Send received data to the RxMessageBuffer
            if (rxpos > 0) xMessageBufferSendFromISR(RxMessageBuffer, &rxbuf, rxpos, NULL);
        } else if (rxByte != NOP) { // If regular data byte
            rxbuf[rxpos++] = rxByte; // Add received byte to the rxbuf
        }

        // Transmit the byte
        if (xQueueReceiveFromISR(SPI0TxQueue, &txByte, &xHigherPriorityTaskWoken) == pdFAIL) {
            txByte=NOP;
        }
        
        // Wait for the transmission to complete
        LPC_SSP0->DR = txByte;
        while (!(LPC_SSP0->SR & (1 << 4)));
    }

    // Re-enable the interrupt mask and enable interrupts
    LPC_SSP0->IMSC |= (1 << 2);
    portENABLE_INTERRUPTS();

    // Yield from ISR if a higher priority task was woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


// Extracts an integer from a delimited string that ends with a letter.
// Returns 0 if the delimiter is not found or if no letter is found after the delimiter.
int32_t p2i(const char *str, const char part) {
    const char C[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    char buf[64];
    char * pch;
    char * sch;
    uint8_t len=0;
    buf[0]=0;
    memset(buf, 0, sizeof(buf));

    pch = strchr(str, part);
    if (pch) {
        sch = (char *) strpbrk(pch+1, C);
        if (sch) {
            len = sch - pch-1;
        } else {
            len = strlen(str) - (size_t) sch;
        }
        strncpy(buf, pch+1, len);
    } else return 0;
    return atoi(buf);
}

void ROSCommsFillSPI0TxQueue_Task(void *pvParameters) {
    char local_txbuf[buflen];
    char txByte;
    size_t txpos;

    char test = 0;

    while (1) {
        // Check if the SPI0TxQueue is empty
        if (uxQueueMessagesWaiting(SPI0TxQueue) == 0) {
            // Receive data from the TxMessageBuffer
            if (xMessageBufferReceive(TxMessageBuffer, &local_txbuf, sizeof(local_txbuf), pdMS_TO_TICKS(10)) > 0) {
                // Enqueue the SOF byte
                txByte = SOF;
                xQueueSend(SPI0TxQueue, &txByte, 0);

                // Enqueue the received data
                for (txpos = 0; txpos < strlen(local_txbuf); txpos++) {
                    xQueueSend(SPI0TxQueue, &local_txbuf[txpos], 0);
                }

                // Enqueue the EOF byte
                txByte = EOF;
                xQueueSend(SPI0TxQueue, &txByte, 0);
            }
        }
        
        //frame++;
        //xQueueSend(SPI0TxQueue, &frame, portMAX_DELAY);

        // Add a delay to allow other tasks to run
        vTaskDelay(xDelay50);
    }
}

void ROSCommsRx_Task(void *pvParameters) {
    char local_rxbuf[buflen];
    RxMessageBuffer = xMessageBufferCreate( 255 );

    for (;;) {
        vTaskDelay(xDelay50);
        while (xMessageBufferReceive(RxMessageBuffer, &local_rxbuf, sizeof(local_rxbuf), 0)) {

            cJSON* root = cJSON_Parse(local_rxbuf);
            if (root != NULL) {
                
                cJSON* obj = cJSON_GetObj(root, "command");
                cJSON* command = obj->child;
                if (command != NULL) {
                   if (!strcasecmp(command->string, "ping")) {
                        debug("Pong! last: %i", watchdogSPI);
                        watchdogSPI=0;
                    } else if (!strcasecmp(command->string, "setpwm")) {
                        debug("Got setpwm\n");
                    } else {
                        debug("unk command: %s", command->string);
                    }
                    
                }
            }

            cJSON_Delete(root);
            


            const char s_SETPWM[] = "SETPWM:";
            const char s_BUTTON[] = "BUTTON:";

            if(strncmp(local_rxbuf, s_SETPWM, sizeof(s_SETPWM) - 1 ) == 0 ) {
                //debug("SETPWM: %s", local_rxbuf);
                xMotorMsgType MotorMsg;
                MotorMsg.action = SETSPEED;
                MotorMsg.pwm.blade = p2i(local_rxbuf, 'B');
                MotorMsg.pwm.left = p2i(local_rxbuf, 'L');
                MotorMsg.pwm.right = p2i(local_rxbuf, 'R');
                xQueueSend(xMotorMsgQueue, &MotorMsg, xDelay25);
            }
            
            if(strncmp(local_rxbuf, s_BUTTON, sizeof(s_BUTTON) - 1 ) == 0 ) {
                xMotorMsgType MotorMsg;
                MotorMsg.action = BUTTON;
                MotorMsg.button.pressed = p2i(local_rxbuf, ':');
                xQueueSend(xMotorMsgQueue, &MotorMsg, xDelay25);
            }
        }
    }
}

void ROSCommsTx_Task(void *pvParameters) {
    vTaskDelay(xDelay1000);
    ROScomms_Init();

    xSensorMsgType sensor;
    xBoundaryMsgType BoundaryMsg;
    HeapStats_t xHeapStats;

    
	//SPI0RxQueue = xQueueCreate(128, sizeof(char));
    
    

    NVIC_SetPriority(SSP0_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    LPC_SSP0->IMSC |= (1 << 1); // 1 RTIM 2 RXIM Rx FIFO is at least half full.  3 TXIM Tx FIFO is at least half empty.
    // Software should write the appropriate control information to the other SSP registers and interrupt controller registers, before setting this bit.
    LPC_SSP0->CR1 |= (1 << 1); // SSP Enable.
    NVIC_EnableIRQ(SSP0_IRQn);
    char local_txbuf[buflen];
    
    uint8_t len = 0;
    uint16_t counter = 0;
    uint8_t printmsg = 0;

    debug("ROSComms started...");

    for (;;) {
        vTaskDelay(xDelay200);
        xJSONMessageType JSONMsg;
        if (xQueueReceive(xJSONMessageQueue, &JSONMsg, 0) == pdPASS) {
            cJSON* root = cJSON_CreateObject();
            cJSON_AddStringToObject(root, JSONMsg.topic, JSONMsg.value);
            cJSON_PrintPreallocated(root, local_txbuf, sizeof(local_txbuf), false);
            xMessageBufferSend(TxMessageBuffer, local_txbuf, strnlen(local_txbuf, sizeof(local_txbuf)) + 1, false);
            cJSON_Delete(root);
        }
        

        cJSON* root = cJSON_CreateObject();
        cJSON* obj = cJSON_CreateObject();
		xQueueReceive(xBoundaryMsgQueue, &BoundaryMsg, xDelay100 );
        
//        len = sprintf(local_txbuf, "Boundary:%s (%i %i %i %i)", (char *) &BoundaryMsg, 
//                atoi(BoundaryMsg.sleft), atoi(BoundaryMsg.sright), atoi(BoundaryMsg.nleft), atoi(BoundaryMsg.nright));
        cJSON_AddItemToObject(obj, "sleft", cJSON_CreateNumber(atoi(BoundaryMsg.sleft)));
        cJSON_AddItemToObject(obj, "sright", cJSON_CreateNumber(atoi(BoundaryMsg.sright)));
        cJSON_AddItemToObject(obj, "nleft", cJSON_CreateNumber(atoi(BoundaryMsg.nleft)));
        cJSON_AddItemToObject(obj, "nright", cJSON_CreateNumber(atoi(BoundaryMsg.nright)));
        cJSON_AddItemToObject(root, "Boundary", obj);    
//        xMessageBufferSend(TxMessageBuffer, local_txbuf, len+1, 0);
        cJSON_PrintPreallocated(root, local_txbuf, sizeof(local_txbuf), false);
        xMessageBufferSend(TxMessageBuffer, local_txbuf, strnlen(local_txbuf, sizeof(local_txbuf)) + 1, false);
        cJSON_Delete(root);

        xQueuePeek(xSensorQueue, &sensor, xDelay100);
        root = cJSON_CreateObject();
        obj = cJSON_CreateObject();
        switch (printmsg) {
            case 1:
                cJSON_AddItemToObject(obj, "mV", cJSON_CreateNumber(sensor.batteryVolt));
                cJSON_AddItemToObject(obj, "mA", cJSON_CreateNumber(sensor.batteryChargeCurrent));
                cJSON_AddItemToObject(obj, "Temp", cJSON_CreateNumber(sensor.batteryTemp));
                cJSON_AddItemToObject(obj, "CellLow", cJSON_CreateNumber(sensor.batteryCellLow));
                cJSON_AddItemToObject(obj, "CellHigh", cJSON_CreateNumber(sensor.batteryCellHigh));
                cJSON_AddItemToObject(obj, "InCharger", cJSON_CreateNumber(sensor.incharger));
                //cJSON_AddItemToObject(obj, "PowerState", cJSON_CreateNumber(powerStates));
                cJSON_AddItemToObject(root, "Battery", obj);
                break;
            case 2:
                cJSON_AddItemToObject(obj, "Stuck", cJSON_CreateNumber(sensor.stuck));
                cJSON_AddItemToObject(obj, "Stuck2", cJSON_CreateNumber(sensor.stuck2));
                cJSON_AddItemToObject(obj, "Door", cJSON_CreateNumber(sensor.door));
                cJSON_AddItemToObject(obj, "Door2", cJSON_CreateNumber(sensor.door2));
                cJSON_AddItemToObject(obj, "Lift", cJSON_CreateNumber(sensor.lift));
                cJSON_AddItemToObject(obj, "Collision", cJSON_CreateNumber(sensor.collision));
                cJSON_AddItemToObject(obj, "Stop", cJSON_CreateNumber(sensor.stop));
                cJSON_AddItemToObject(obj, "Rain", cJSON_CreateNumber(sensor.rain));
                cJSON_AddItemToObject(root, "Digital", obj);
                break;
            case 3:
                cJSON_AddItemToObject(obj, "RainAnalog", cJSON_CreateNumber(sensor.rainAnalog));
                cJSON_AddItemToObject(obj, "boardTemp", cJSON_CreateNumber(sensor.boardTemp)); // RAW
                cJSON_AddItemToObject(root, "Analog", obj);
                break;
            case 4:
                cJSON_AddItemToObject(obj, "PwmLeft", cJSON_CreateNumber(LPC_PWM1->MR4));
                cJSON_AddItemToObject(obj, "PwmRight", cJSON_CreateNumber(LPC_PWM1->MR5));
                cJSON_AddItemToObject(obj, "PwmBlade", cJSON_CreateNumber(LPC_PWM1->MR1));
                cJSON_AddItemToObject(root, "MotorPWM", obj);
                break;
            case 5:
                cJSON_AddItemToObject(obj, "Left", cJSON_CreateNumber(sensor.motorpulseleft));
                cJSON_AddItemToObject(obj, "Right", cJSON_CreateNumber(sensor.motorpulseright));
                cJSON_AddItemToObject(obj, "Blade", cJSON_CreateNumber(sensor.motorpulseblade));
                cJSON_AddItemToObject(root, "MotorPulse", obj);
                break;
            case 6:
                cJSON_AddItemToObject(obj, "Left", cJSON_CreateNumber(sensor.motorRCurrent));
                cJSON_AddItemToObject(obj, "Right", cJSON_CreateNumber(sensor.motorLCurrent));
                cJSON_AddItemToObject(obj, "BladeRPM", cJSON_CreateNumber(sensor.motorBRpm));
                cJSON_AddItemToObject(root, "MotorCurrent", obj);
                break;
            case 7:
                cJSON_AddItemToObject(obj, "Yaw", cJSON_CreateNumber(sensor.GyroYaw));
                cJSON_AddItemToObject(obj, "Pitch", cJSON_CreateNumber(sensor.GyroPitch));
                cJSON_AddItemToObject(obj, "Roll", cJSON_CreateNumber(sensor.GyroRoll));
                cJSON_AddItemToObject(root, "I2C_L3GD20", obj);
                break;
            case 8:
                cJSON_AddItemToObject(obj, "AccelX", cJSON_CreateNumber(sensor.AccelX));
                cJSON_AddItemToObject(obj, "AccelY", cJSON_CreateNumber(sensor.AccelY));
                cJSON_AddItemToObject(obj, "AccelZ", cJSON_CreateNumber(sensor.AccelZ));
                cJSON_AddItemToObject(root, "I2C_MMA8452Q", obj);
                break;
            case 9:
                int spibus = SystemCoreClock / (LPC_SC->PCLKSEL * (LPC_SSP0->CPSR * (1 + (LPC_SSP0->CR0 >> 8))));
                cJSON_AddItemToObject(obj, "CpuClk", cJSON_CreateNumber(SystemCoreClock));
                cJSON_AddItemToObject(obj, "SpiClk", cJSON_CreateNumber(spibus));
                cJSON_AddItemToObject(root, "Debug", obj);
                printmsg = 0;
                break;
        }
        printmsg++;

        cJSON_PrintPreallocated(root, local_txbuf, sizeof(local_txbuf), false);
        xMessageBufferSend(TxMessageBuffer, local_txbuf, strnlen(local_txbuf, sizeof(local_txbuf)) + 1, false);
        cJSON_Delete(root);
        
        if (!(counter % 40)) {
            vTaskDelay(xDelay200);
            for ( int i = 0; i < taskcounter; i++) {
                len = sprintf(local_txbuf,"Task %i:%s StackHigh: %li ",i , pcTaskGetName(xHandle[i]), uxTaskGetStackHighWaterMark(xHandle[i]));
                xMessageBufferSend(TxMessageBuffer, local_txbuf, len+1, 0);    
            }
            vPortGetHeapStats( &xHeapStats );
            len = sprintf(local_txbuf, "HeapStats: Free:%i (min: %i) Allocs: %i Frees: %i", xHeapStats.xAvailableHeapSpaceInBytes, 
                xHeapStats.xMinimumEverFreeBytesRemaining, xHeapStats.xNumberOfSuccessfulAllocations, xHeapStats.xNumberOfSuccessfulFrees); 
            xMessageBufferSend(TxMessageBuffer, local_txbuf, len+1, 0);
        }
        counter++;
        int stack = uxTaskGetStackHighWaterMark(NULL);
        if (stack < 50) debug("Task ROSComms_Task has %u words left in stack.", stack);
    }
}

void debug( const char* format, ... ) {
    char buf[buflen];
    uint8_t len;
    va_list args;
    len = sprintf( buf, "DEBUG: " );
    va_start( args, format );
    len += vsprintf( buf + len, format, args );
    va_end( args );
    //len += sprintf( buf + len, "\n" );
    buf[len++]=0;
    xMessageBufferSend(TxMessageBuffer, buf, len, 0);
}

#pragma import(__use_no_semihosting_swi)

__attribute__((used)) int _write(int fd, char *ptr, int len) {
    int i = 0;
    char ch;
    while (*ptr && (i < len)) {
        ch = *ptr;
        xQueueSend(SPI0TxQueue, (char*)&ch, (TickType_t)0);
        i++;
        ptr++;
    }

    return i;
}
