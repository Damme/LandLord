#include "u8g.h"

#ifndef LCD_H
#define LCD_H

// TODO: fix better defines with comments. ---> define.h
#define rstb 19
#define csb  16
#define a0   20

#define sclk 15
#define sda  18


void delayMs(uint8_t timer_num, uint32_t delayInuS);
void delay_system_ticks(uint32_t sys_ticks);
void delay_micro_seconds(uint32_t us);
void spi_out(uint8_t data);
uint8_t u8g_com_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);

#define SYS_TICK_PERIOD_IN_MS 10 // or?

void task_LCD(void *pvParameters);

#endif
