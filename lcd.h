#include "u8g.h"

#ifndef LCD_H
#define LCD_H

uint8_t u8g_com_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);

void LCD_Task(void *pvParameters);

#endif
