#include <stdio.h>
#include "lcd.h"
#include "screen.h"
#include "global.h"

#include "FreeRTOS.h"
#include "timers.h"

void spi_out(uint8_t data) {
#ifdef LPC177x_8x // DB504
    // db504 does not use hardware ssp pins so we have to bitbang instead..
    for (uint8_t i = 0; i < 8; i++) {
        // consider leftmost bit
        // set line high if bit is 1, low if bit is 0
        if (data & 0x80)
            GPIO_SET_PIN(LCD_SDA);
        else
            GPIO_CLR_PIN(LCD_SDA);
        // pulse clock to indicate that bit value should be read
        GPIO_SET_PIN(LCD_SCLK);
        GPIO_CLR_PIN(LCD_SCLK);
        // shift byte left so next bit will be leftmost
        data <<= 1;
    }
#else
    LPC_SPI->SPDR = data;
    while (((LPC_SPI->SPSR >> 7) & 1) == 0);
#endif

}

void u8g_Delay(uint16_t val) {
    delay_uS(1000UL * (uint32_t)val);
}

void u8g_MicroDelay(void) {
    delay_uS(1);
}

void u8g_10MicroDelay(void) {
    delay_uS(10);
}

uint8_t u8g_com_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr) {
    switch (msg) {
        case U8G_COM_MSG_STOP:
            break;

        case U8G_COM_MSG_INIT:
            // init spi and ports
            u8g_MicroDelay();
            break;

        case U8G_COM_MSG_ADDRESS:                     /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
            //u8g_10MicroDelay();
            GPIO_SET_PIN_VAL(LCD_A0, arg_val);
            u8g_MicroDelay();
            break;

        case U8G_COM_MSG_CHIP_SELECT:
            GPIO_SET_PIN_VAL(LCD_CSB, !arg_val);

            u8g_MicroDelay();
            break;

        case U8G_COM_MSG_RESET:
            GPIO_SET_PIN_VAL(LCD_RSTB, arg_val);
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

void LCD_Task(void *pvParameters) {
    TickType_t xLastTime;
    xLastTime = xTaskGetTickCount();

    // Configure LCD backligt
    GPIO_DIR_OUT(LCD_BACKLIGHT);
    GPIO_SET_PIN(LCD_BACKLIGHT);

    // Configure SPI
    GPIO_DIR_OUT(LCD_RSTB);
    GPIO_DIR_OUT(LCD_CSB);
    GPIO_DIR_OUT(LCD_A0);

    // Configure Timer1 used for µs delay in lcd
#ifdef LPC175x_6x
    LPC_SC->PCONP |= PCONP_PCTIM1;              // power up Timer (def on)
    LPC_SC->PCLKSEL0 |= PCLK_TIMER1(CCLK_DIV1); // set Timer0 clock1

    // Configure SPI (LCD)
    LPC_SC->PCONP |= PCONP_PCSPI;               // power up SPI
    LPC_SC->PCLKSEL0 |= PCLK_SPI(CCLK_DIV1);    // set SPI CCLK

    LPC_PINCON->PINSEL0 |= ((uint32_t)3 << 30); // p0.15 -> sck
    LPC_PINCON->PINSEL1 |= (0xc | 0x30);        // p0.17 & p0.18 miso / mosi (no miso??)

    LPC_SPI->SPCR |= SPCR_MSTR;                 // SPI operates in Master mode.
#endif

    LCD_Init();

    for (;;) {
        vTaskDelayUntil(&xLastTime, xDelay100);
        screen_Task();
#if LOWSTACKWARNING
        int stack = uxTaskGetStackHighWaterMark(NULL);
        if (stack < 50) printf("Task task_LCD has %u words left in stack.\r\n", stack);
#endif

    }
}
