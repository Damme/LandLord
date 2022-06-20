#include <global.h>

//int stack_Keypad, stack_Counter, stack_LCD, stack_ADC = 0;

TickType_t xDelay1 = TicksPerMS * 1;
TickType_t xDelay10 = TicksPerMS * 10;
TickType_t xDelay25 = TicksPerMS * 25;
TickType_t xDelay50 = TicksPerMS * 50;
TickType_t xDelay100 = TicksPerMS * 100;
TickType_t xDelay200 = TicksPerMS * 200;
TickType_t xDelay150 = TicksPerMS * 150;
TickType_t xDelay250 = TicksPerMS * 250;
TickType_t xDelay500 = TicksPerMS * 500;
TickType_t xDelay1000 = TicksPerMS * 1000;


volatile uint32_t debug1 = 0;
volatile uint8_t debug2 = 0;
volatile uint8_t debug3 = 0;
volatile uint8_t debug4 = 0;
volatile debug_t debugArray[5] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
volatile uint8_t debugState = 0;

volatile uint32_t cpuID = 0;

volatile TaskHandle_t xHandle[10] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };
volatile uint8_t taskcounter=0;

xQueueHandle xScreenMsgQueue;
xQueueHandle xMotorMsgQueue;
xQueueHandle xSensorQueue;

xQueueHandle xBoundaryMsgQueue;


void delay_uS(uint32_t uS) {
    portENTER_CRITICAL();
    LPC_TIM1->TCR = 0x02;                // reset timer
    LPC_TIM1->PR  = 0x00;                // set prescaler to zero
    LPC_TIM1->MR0 = uS * (SystemCoreClock / 1000000) - 1;
    LPC_TIM1->IR  = 0xff;                // reset all interrrupts
    LPC_TIM1->MCR = 0x04;                // stop timer on match
    LPC_TIM1->TCR = 0x01;                // start timer
    // wait until delay time has elapsed
    while (LPC_TIM1->TCR & 0x01);
    portEXIT_CRITICAL();
}

void vAssertCalled( void ) {
    volatile unsigned long looping = 0;
    GPIO_TGL_PIN(LCD_BACKLIGHT);

    taskENTER_CRITICAL();
    {
        /* Use the debugger to set ul to a non-zero value in order to step out
         *      of this function to determine why it was called. */
        while( looping == 0LU ) {
            portNOP();
        }
    }
    taskEXIT_CRITICAL();
}
/*
typedef void (*IAP)(uint32_t[5], uint32_t[5]);
const IAP IAP_entry = (IAP)0x1FFF1FF1;

int iap_read_id(void)
{
    unsigned int cmd[5], res[5];
    cmd[0] = 54;
    IAP_entry(cmd, res);
    return ((int)res[1]);
}
*/
