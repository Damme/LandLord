#include "lcd.h"

void delayMs(uint8_t timer_num, uint32_t delayInuS)
{
    /*    TODO: Fix alternative delay for u8g - without using system timer.
        systick-handle counts down 'lcdcounter' to 0 if > 0.
        code can set lcd-counter and wait til zero.
     */

    if (timer_num == 0) {
        LPC_TIM0->TCR = 0x02;                /* reset timer */
        LPC_TIM0->PR  = 0x00;                /* set prescaler to zero */
        LPC_TIM0->MR0 = delayInuS * (SystemCoreClock / 1000000); // 23980 = 1ms
        LPC_TIM0->IR  = 0xff;                /* reset all interrrupts */
        LPC_TIM0->MCR = 0x04;                /* stop timer on match */
        LPC_TIM0->TCR = 0x01;                /* start timer */

        /* wait until delay time has elapsed */
        while (LPC_TIM0->TCR & 0x01);
    } else if (timer_num == 1) {
        LPC_TIM1->TCR = 0x02;                /* reset timer */
        LPC_TIM1->PR  = 0x00;                /* set prescaler to zero */
        LPC_TIM1->MR0 = delayInuS * (SystemCoreClock / 1000000); // 23980 = 1ms
        LPC_TIM1->IR  = 0xff;                /* reset all interrrupts */
        LPC_TIM1->MCR = 0x04;                /* stop timer on match */
        LPC_TIM1->TCR = 0x01;                /* start timer */

        /* wait until delay time has elapsed */
        while (LPC_TIM1->TCR & 0x01);
    } else if (timer_num == 2) {
        LPC_TIM2->TCR = 0x02;                /* reset timer */
        LPC_TIM2->PR  = 0x00;                /* set prescaler to zero */
        LPC_TIM2->MR0 = delayInuS * (SystemCoreClock / 1000000); // 23980 = 1ms
        LPC_TIM2->IR  = 0xff;                /* reset all interrrupts */
        LPC_TIM2->MCR = 0x04;                /* stop timer on match */
        LPC_TIM2->TCR = 0x01;                /* start timer */

        /* wait until delay time has elapsed */
        while (LPC_TIM2->TCR & 0x01);
    } else if (timer_num == 3) {
        LPC_TIM3->TCR = 0x02;                /* reset timer */
        LPC_TIM3->PR  = 0x00;                /* set prescaler to zero */
        LPC_TIM3->MR0 = delayInuS * (SystemCoreClock / 1000000); // 23980 = 1ms
        LPC_TIM3->IR  = 0xff;                /* reset all interrrupts */
        LPC_TIM3->MCR = 0x04;                /* stop timer on match */
        LPC_TIM3->TCR = 0x01;                /* start timer */

        /* wait until delay time has elapsed */
        while (LPC_TIM3->TCR & 0x01);
    }
}
void delay_micro_seconds(uint32_t us)
{
    delayMs(1, us);
}

void spi_out(uint8_t data)
{
    LPC_SPI->SPDR = data;
    while (((LPC_SPI->SPSR >> 7) & 1) == 0);
}


void u8g_Delay(uint16_t val)
{
    delay_micro_seconds(1000UL * (uint32_t)val);
}

void u8g_MicroDelay(void)
{
    delay_micro_seconds(1);
}

void u8g_10MicroDelay(void)
{
    delay_micro_seconds(10);
}

uint8_t u8g_com_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
    switch (msg) {
        case U8G_COM_MSG_STOP:
            break;

        case U8G_COM_MSG_INIT:
            // init spi and ports
            u8g_MicroDelay();
            break;

        case U8G_COM_MSG_ADDRESS:                     /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
            //u8g_10MicroDelay();
            if (arg_val) { LPC_GPIO0->FIOPIN |= (1 << a0); } else { LPC_GPIO0->FIOPIN &= ~(1 << a0); }
            u8g_MicroDelay();
            break;

        case U8G_COM_MSG_CHIP_SELECT:
            if (!arg_val) { LPC_GPIO0->FIOPIN |= (1 << csb); } else { LPC_GPIO0->FIOPIN &= ~(1 << csb); }
            u8g_MicroDelay();
            break;

        case U8G_COM_MSG_RESET:
            if (arg_val) { LPC_GPIO0->FIOPIN |= (1 << rstb); } else { LPC_GPIO0->FIOPIN &= ~(1 << rstb); }
            u8g_MicroDelay();
            break;

        case U8G_COM_MSG_WRITE_BYTE:
            spi_out(arg_val);
            u8g_MicroDelay();
            break;

        case U8G_COM_MSG_WRITE_SEQ:
        case U8G_COM_MSG_WRITE_SEQ_P: {
                register uint8_t *ptr = arg_ptr;
                while (arg_val > 0) {
                    spi_out(*ptr++);
                    arg_val--;
                }
            }
            break;
    }
    return 1;
}