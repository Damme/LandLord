#include <global.h>

int stack_Keypad, stack_Counter, stack_LCD, stack_ADC = 0;


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
volatile uint32_t ADC0 = 0;
volatile uint32_t ADC1 = 0;
volatile uint32_t ADC2 = 0;
volatile uint32_t ADC3 = 0;
volatile uint32_t ADC4 = 0;
volatile uint32_t ADC5 = 0;
volatile uint32_t ADC6 = 0;
volatile uint32_t ADC7 = 0;

__attribute__((used)) int _write(int fd, char *ptr, int len)
{
    int i = 0;
    while (*ptr && (i < len)) {
        ITM_SendChar(*ptr);
        i++;
        ptr++;
    }
    return i;
}

void delayuS(uint32_t uS)
{
    LPC_TIM1->TCR = 0x02;                /* reset timer */
    LPC_TIM1->PR  = 0x00;                /* set prescaler to zero */
    LPC_TIM1->MR0 = uS * (SystemCoreClock / 1000000) - 1; // 23980 = 1ms
    LPC_TIM1->IR  = 0xff;                /* reset all interrrupts */
    LPC_TIM1->MCR = 0x04;                /* stop timer on match */
    LPC_TIM1->TCR = 0x01;                /* start timer */

    /* wait until delay time has elapsed */
    while (LPC_TIM1->TCR & 0x01);
}

