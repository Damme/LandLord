#include "lcd.h"

void spi_out(uint8_t data)
{
    LPC_SPI->SPDR = data;
    while (((LPC_SPI->SPSR >> 7) & 1) == 0);
}


void u8g_Delay(uint16_t val)
{
    delayuS(1000UL * (uint32_t)val);
}

void u8g_MicroDelay(void)
{
    delayuS(1);
}

void u8g_10MicroDelay(void)
{
    delayuS(10);
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
