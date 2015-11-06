#include "LPC17xx.h"
#include "u8g.h"
#include <stdint.h>


//todo: fix better defines with comments.
#define rstb 19
#define csb  16
#define a0   20


u8g_t u8g;



void delay_system_ticks(uint32_t sys_ticks);	
void delay_micro_seconds(uint32_t us);
void spi_out(uint8_t data);
uint8_t u8g_com_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr); 

#define SYS_TICK_PERIOD_IN_MS 10