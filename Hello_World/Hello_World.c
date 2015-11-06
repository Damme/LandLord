#include "Hello_World.h" 

int main(void) {
  
// Setup Systick
  SysTick->LOAD = (SystemCoreClock/1000UL*(unsigned long)SYS_TICK_PERIOD_IN_MS) - 1;
  SysTick->VAL = 0;
  SysTick->CTRL = 7; // enable, generate interrupt (SysTick_Handler), do not divide by 2

// Power cntrl
  LPC_SC->PCONP |= (1 << 15);   // power up GPIO
  LPC_SC->PCONP |= (1 << 8);    // power up SPI

// Keep Pwr ON
  LPC_GPIO1->FIODIR |= ( 1 << 25 ); // p1.25 output mode.
  LPC_GPIO1->FIOPIN |= ( 1 << 25 ); // p1.25 PWR ON??

// Configure SPI (LCD)
  LPC_SC->PCLKSEL0 |= (1 << 16);  // set SPI CCLK

  LPC_GPIO1->FIODIR |= (1 << 20);   // P1.20 output mode.
  LPC_GPIO0->FIODIR |= (1 << rstb); // p0.19 output mode.
  LPC_GPIO0->FIODIR |= (1 << csb);  // p0.16 output mode.
  LPC_GPIO0->FIODIR |= (1 << a0);   // p0.20 output mode.

  LPC_PINCON->PINSEL0 |= (1 << 30) | (1 << 31); // p0.15 -> sck
  LPC_PINCON->PINMODE0 |= (1 << 30);            // p0.15 Repeater mode *todo: need more checking

  LPC_PINCON->PINSEL1 |= ( 0xc | 0x30 );  // p0.17 & p0.18 miso / mosi (no miso??)
  LPC_PINCON->PINMODE1 |= ( (1 << 0) | (1 << 2) | (1 << 4));  // p0.16 p0.17 p0.18  repeater mode *todo: need more checking
  LPC_PINCON->PINMODE1 |= (1 << 6);   // p0.19 repeater mode *todo: need more checking
 
  LPC_SPI->SPCR |= (1 << 5);  // SPI operates in Master mode.


//Configur u8g
  //u8g_InitComFn(&u8g, &u8g_dev_st7565_nhd_c12864_hw_spi, u8g_com_hw_spi_fn);
  u8g_InitComFn(&u8g, &u8g_dev_st7565_nhd_c12864_2x_hw_spi, u8g_com_hw_spi_fn); 
  u8g_SetContrast(&u8g, 4 );
  u8g_SetDefaultForegroundColor(&u8g);
  u8g_SetRot180(&u8g);
// Turn on LCD backlight
  LPC_GPIO1->FIOPIN |= ( 1 << 20 );  // p1.20 LCD backlight ON
 



  for (;;){
    u8g_FirstPage(&u8g);
    do {
      u8g_SetFont(&u8g, u8g_font_tpss);
      u8g_DrawStr(&u8g,  0, 25, "Hello World!");
    } while ( u8g_NextPage(&u8g) );
    delay_micro_seconds(250000);
  }
}
