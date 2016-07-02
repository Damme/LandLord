#include "lcd.h"
#include "screen.h"
#include "define.h"

#include "FreeRTOS.h"
#include "timers.h"

#define xDelay25   ((TickType_t)25 / portTICK_PERIOD_MS)
#define xDelay100  ((TickType_t)100 / portTICK_PERIOD_MS)

void spi_out(uint8_t data)
{
    LPC_SPI->SPDR = data;
    while (((LPC_SPI->SPSR >> 7) & 1) == 0);
}


void u8g_Delay(uint16_t val)
{
    vTaskDelay((uint32_t)val / portTICK_PERIOD_MS);
}

void u8g_MicroDelay(void)
{
    vTaskDelay(1 / portTICK_PERIOD_MS);
}

void u8g_10MicroDelay(void)
{
    vTaskDelay(10 / portTICK_PERIOD_MS);
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
            if (arg_val) { LPC_GPIO0->FIOSET = (1 << a0); } else { LPC_GPIO0->FIOCLR = (1 << a0); }
            u8g_MicroDelay();
            break;

        case U8G_COM_MSG_CHIP_SELECT:
            if (!arg_val) { LPC_GPIO0->FIOSET = (1 << csb); } else { LPC_GPIO0->FIOCLR = (1 << csb); }
            u8g_MicroDelay();
            break;

        case U8G_COM_MSG_RESET:
            if (arg_val) { LPC_GPIO0->FIOSET = (1 << rstb); } else { LPC_GPIO0->FIOCLR = (1 << rstb); }
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

static void task_LCD(void *pvParameters)
{
    TickType_t xLastTime;
    xLastTime = xTaskGetTickCount();

	  // Configure LCD backligt
    LPC_GPIO1->FIODIR |= PIN(20);               // P1.20 output mode.
    LPC_GPIO1->FIOPIN |= PIN(20);               // p1.20 LCD backlight ON

    // Configure SPI (LCD)
    LPC_SC->PCONP |= PCONP_PCSPI;               // power up SPI
    LPC_SC->PCLKSEL0 |= PCLK_SPI(CCLK_DIV1);    // set SPI CCLK

    LPC_GPIO0->FIODIR |= PIN(rstb);             // p0.19 output mode.
    LPC_GPIO0->FIODIR |= PIN(csb);              // p0.16 output mode.
    LPC_GPIO0->FIODIR |= PIN(a0);               // p0.20 output mode.

    LPC_PINCON->PINSEL0 |= ((uint32_t)3 << 30); // p0.15 -> sck
    LPC_PINCON->PINSEL1 |= (0xc | 0x30);        // p0.17 & p0.18 miso / mosi (no miso??)

    LPC_SPI->SPCR |= SPCR_MSTR;                 // SPI operates in Master mode.

		for (;;) {
        vTaskDelayUntil(&xLastTime, xDelay100);
        lcdUpdate();
    }
}
