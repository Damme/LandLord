#include "ROSComms.h"
#include "global.h"
#include "common.h"
#include "queue.h"
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "task.h"
#include <stdarg.h>

xQueueHandle SPI0TxQueue;
xQueueHandle SPI0RxQueue;

MessageBufferHandle_t RxMessageBuffer;
MessageBufferHandle_t TxMessageBuffer;

#define buflen  192

volatile uint8_t frame = 0;
volatile uint8_t oldframe = 0;
volatile uint8_t rxpos = 0;
volatile uint8_t rxbuf[buflen];

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
            // maybe replace SPI0TxQueue
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

int32_t p2i(const char *str, const char part) {
    const char C[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    char buf[128];
    char * pch;
    char * sch;
    uint8_t len=0;
    buf[0]=0;

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


void ROSComms_Task(void *pvParameters) {
    vTaskDelay(xDelay1000);
    ROScomms_Init();
    xSensorMsgType sensor;
    HeapStats_t xHeapStats;

    SPI0TxQueue = xQueueCreate(buflen, sizeof(char));
	//SPI0RxQueue = xQueueCreate(128, sizeof(char));
    RxMessageBuffer = xMessageBufferCreate( 256 );
    TxMessageBuffer = xMessageBufferCreate( 1024 );

    NVIC_SetPriority(SSP0_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    
    LPC_SSP0->IMSC |= (1 << 2); // 1 RTIM 2 RXIM Rx FIFO is at least half full.  3 TXIM Tx FIFO is at least half empty.
    // Software should write the appropriate control information to the other SSP registers and interrupt controller registers, before setting this bit.
    LPC_SSP0->CR1 |= (1 << 1); // SSP Enable.
    NVIC_EnableIRQ(SSP0_IRQn);
    char local_txbuf[buflen];
    char local_rxbuf[buflen];
    uint8_t len = 0;
    uint16_t counter = 0;
 
 //   printf("ROSComms started...\n");

    for (;;) {
        vTaskDelay(xDelay50);
        
        while (xMessageBufferReceive(RxMessageBuffer, &local_rxbuf, sizeof(local_rxbuf), 0)) {
            if (strcmp(local_rxbuf,"Ping?") == 0) {
                debug("Pong!");
            }

            const char s_SETPWM[] = "SETPWM:";

            if(strncmp(local_rxbuf, s_SETPWM, sizeof(s_SETPWM) - 1 ) == 0 ) {
                debug("SETPWM! :D L:%i R:%i B:%i", p2i(local_rxbuf, 'L'), p2i(local_rxbuf, 'R'), p2i(local_rxbuf, 'B'));
                xMotorMsgType MotorMsg;
                MotorMsg.action = SETSPEED;
                MotorMsg.blade = 0;
                MotorMsg.left = p2i(local_rxbuf, 'L');
                MotorMsg.right = p2i(local_rxbuf, 'R');
                xQueueSend(xMotorMsgQueue, &MotorMsg, xDelay25);
            }
        }

        if (!(counter % 25)) {
            xQueuePeek(xSensorQueue, &sensor, TicksPerMS*10);
            len = sprintf(local_txbuf, "Batt: mV:%li mA:%li Temp:%li BS:%i BH:%i InCharger:%i", sensor.batteryVolt, 
                sensor.batteryChargeCurrent, sensor.batteryTemp, sensor.batteryCellLow, sensor.batteryCellHigh, sensor.incharger);
            xMessageBufferSend(TxMessageBuffer, local_txbuf, len+1, 0);

            len = sprintf(local_txbuf, "Sensors Digital: Stuck:%i,%i Door:%i,%i Lift:%i Coll:%i Stop:%i Rain:%i", sensor.stuck, sensor.stuck2,
                sensor.door, sensor.door2, sensor.lift, sensor.collision, sensor.stop, sensor.rain);
            xMessageBufferSend(TxMessageBuffer, local_txbuf, len+1, 0);

            len = sprintf(local_txbuf, "Sensors Other: RainAnalog:%li boardTemp(raw):%li", sensor.rainAnalog, sensor.boardTemp, sensor.MotionRoll);
            xMessageBufferSend(TxMessageBuffer, local_txbuf, len+1, 0);

            len = sprintf(local_txbuf, "Motor: PwmLeft:%li PwmRight:%li PwmBlade:%li motorRCurrent:%li motorLCurrent:%li motorSCurrent:%li", LPC_PWM1->MR4, LPC_PWM1->MR5, LPC_PWM1->MR1, sensor.motorRCurrent, sensor.motorLCurrent, sensor.motorSCurrent);
            xMessageBufferSend(TxMessageBuffer, local_txbuf, len+1, 0);

            len = sprintf(local_txbuf, "I2C L3GD20 Motion: Yaw:%li Pitch:%li Roll:%li", sensor.MotionYaw, sensor.MotionPitch, sensor.MotionRoll);
            xMessageBufferSend(TxMessageBuffer, local_txbuf, len+1, 0);

            len = sprintf(local_txbuf, "I2C MMA8452Q Accel: AccelX:%li AccelY:%li AccelZ:%li", sensor.AccelX, sensor.AccelY, sensor.AccelZ);
            xMessageBufferSend(TxMessageBuffer, local_txbuf, len+1, 0);

            len = sprintf(local_txbuf, "I2C LSM303 Mag: MagX:%li MagY:%li MagZ:%li", sensor.MagX, sensor.MagY, sensor.MagZ);
            xMessageBufferSend(TxMessageBuffer, local_txbuf, len+1, 0);


        }

        // print task statistics every 600 ticks.
        if (!(counter % 610)) {
            vTaskDelay(xDelay200);
            for ( int i = 0; i < taskcounter; i++) {
                len = sprintf(local_txbuf,"Task %i:%s free heap:%i",i , pcTaskGetName(xHandle[i]), uxTaskGetStackHighWaterMark(xHandle[i]));
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

        char boundbuf[128];
		xQueueReceive(xBoundaryMsgQueue, &boundbuf, xDelay25 );

        len = sprintf(local_txbuf, "* %i: %s", debug1, boundbuf);
        xMessageBufferSend(TxMessageBuffer, local_txbuf, len+1, 0);
        
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
