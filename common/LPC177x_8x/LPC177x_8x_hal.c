#include "LPC177x_8x_hal.h"
#include "global.h"

void hardware_Init() {
    LPC_SC->PCONP |= PCONP_PCGPIO;              // Power up GPIO
    LPC_SC->PCONP |= PCONP_PCADC;               // Power up ADC
    
    // Keep power on
    GPIO_DIR_OUT(POWER);
    GPIO_SET_PIN(POWER);
    
    // TODO: Turn on lcd backlight - Move to screen, and insert timer, and pwm.
    GPIO_DIR_OUT(LCD_BACKLIGHT);
    GPIO_SET_PIN(LCD_BACKLIGHT);

    // Configure Power button
    GPIO_PIN_FNC(KEYPAD_POWER);
    GPIO_FNC_PULL(KEYPAD_POWER, PINMODE_PULLDOWN);

    // Pulse charger to keep it in "red mode"
    GPIO_DIR_OUT(CHARGER_ENABLE);
    GPIO_SET_PIN(CHARGER_ENABLE);
    delay_uS(10000);
    GPIO_CLR_PIN(CHARGER_ENABLE);    
    
    GPIO_DIR_OUT(BUZZER_LO);
    GPIO_DIR_OUT(BUZZER_HI);

    GPIO_SET_PIN(BUZZER_LO);
    for (uint16_t i = 0; i < 50; i++) delay_uS(1000); // Todo: maybe use timer in future? 
    GPIO_CLR_PIN(BUZZER_LO); 
}

void sensor_Init() {
    uint32_t fullAdcRate;
    uint8_t div;
    //fullAdcRate = 400000 * 31;
    fullAdcRate = 375000 * 33;
    div = ((PeripheralClock * 2 + fullAdcRate) / (fullAdcRate * 2)) - 1;
/*
    GPIO_PIN_FNC(ADC_AD0);
    GPIO_PIN_FNC(ADC_AD1);
    GPIO_PIN_FNC(ADC_AD2);
    GPIO_PIN_FNC(ADC_AD3);
    GPIO_PIN_FNC(ADC_AD4);
    GPIO_PIN_FNC(ADC_AD5);
    GPIO_PIN_FNC(ADC_AD6);
    GPIO_PIN_FNC(ADC_AD7);
*/  
    LPC_IOCON->P0_12 = (1 << 8) | (3 << 0);
    LPC_IOCON->P0_13 = (1 << 8) | (3 << 0);
    LPC_IOCON->P0_23 = (1 << 8) | (1 << 0);
    LPC_IOCON->P0_24 = (1 << 8) | (1 << 0);
    LPC_IOCON->P0_25 = (1 << 8) | (1 << 0);
    LPC_IOCON->P0_26 = (1 << 8) | (1 << 0);
    LPC_IOCON->P1_30 = (1 << 8) | (3 << 0);
    LPC_IOCON->P1_31 = (1 << 8) | (3 << 0);
    
    LPC_ADC->CR = (div << 8) | (1 << 21); //div 4 400000hz, enable ADC ändrat till div 8
    LPC_ADC->CR |= ( 0xff ); // enable all adc channels.
    LPC_ADC->CR |= (1UL<<16); // enable Burst mode
}

void powerMgmt_Init() {
    GPIO_DIR_OUT(CHARGER_CHECK);
    GPIO_DIR_OUT(CHARGER_ENABLE);
    GPIO_FNC_INV(CHARGER_CONNECTED, PINMODE_INV );
}

void ROScomms_Init() {
    // Init hardware 1788
    LPC_SC->PCONP |= PCONP_PCSPP0;              // power up SSP0
    
    GPIO_PIN_FNC(SSP0_SCK);
    GPIO_PIN_FNC(SSP0_SSEL);
    GPIO_PIN_FNC(SSP0_MISO);
    GPIO_PIN_FNC(SSP0_MOSI);
    
    GPIO_DIR_IN(SSP0_SSEL);
    //GPIO_SET_PIN(SSP0_SSEL);
    GPIO_FNC_PULL(SSP0_SCK, PINMODE_PULLDOWN);
    
    LPC_SSP0->CR0 = (7 << 0) | (1 << 6) | (1 << 7); // 8bits, CPOL, CPHA,
    LPC_SSP0->CR1 = (1 << 2);

    LPC_SSP0->CPSR = (8 & 0xfe); // SSPn Clock Prescale Register 60000000 / 128 = 468750
    LPC_SSP0->CR0 |= (6 << 8); // Serial Clock Rate.
    //LPC_SSP0->CR1 |= (1 << 0); // loopback
    LPC_SSP0->CR1 |= (1 << 2); // Slave mode
    
    // PCLK / (CPSDVSR × [SCR+1]) = bitrate

//  SCR \ SCPR
//  	8	        16	        32	        64	        128	        256
//  1	3,750,000	1,875,000	937,500	    468,750	    234,375	    117,188
//  2	2,500,000	1,250,000	625,000	    312,500	    156,250	    78,125
//  3	1,875,000	937,500	    468,750	    234,375	    117,188	    58,594
//  4	1,500,000	750,000	    375,000	    187,500	    93,750	    46,875
//  5	1,250,000	625,000	    312,500	    156,250	    78,125	    39,063
//  6	1,071,429	535,714	    267,857	    133,929	    66,964	    33,482
//  7	937,500	    468,750	    234,375	    117,188	    58,594	    29,297
//  8	833,333	    416,667	    208,333	    104,167	    52,083	    26,042
//  9	750,000	    375,000	    187,500	    93,750	    46,875	    23,438
//  10	681,818	    340,909	    170,455	    85,227	    42,614	    21,307

    // Set DSS data to 8-bit, Frame format SPI, CPOL = 0, CPHA = 0, and SCR is 15
    //LPC_SSP0->CR0 = 0x07c7; // 707 = 00, 747 = 01 787 = 10 7c7 = 11
    // SSPCPSR clock prescale register, master mode, minimum divisor is 0x02
    //LPC_SSP0->CPSR = 0x2;
    // 60 000 000 / ( 2 x 16 ) -> 1875000

    /*if ( LPC_SSP0->CR1 & (1 << 1) ) {
	// The slave bit can't be set until SSE bit is zero. 
	    LPC_SSP0->CR1 &= ~(1 << 1);
    }
    LPC_SSP0->CR1 = (1 << 2);	// Enable slave bit first 
    LPC_SSP0->CR1 |= (1 << 1);	// Enable SSP 
*/
}

void MotorCtrl_Init() {
    LPC_SC->PCONP |= PCONP_PCPWM1;

// Configure PWM 1.1(Blade) 1.4(left) 1.5(right) (and pwm1.2 LCD brightness)
    LPC_PWM1->MR0 = 2047;
    LPC_PWM1->MR1 = 0;      // Blade
    LPC_PWM1->MR4 = 0;      // Left
    LPC_PWM1->MR5 = 0;      // Right
    LPC_PWM1->MR2 = 2047;   // LCD
// Setup PWM registers
    // mr0 = 1000 pr = 59 : ~1k hz
    // mr0 = 1000 pr = 12 : ~4.6 khz
    // mr0 = 1000 pr = 5  : ~10 khz
    // mr0 = 1000 pr = 2  : ~20 khz
    LPC_PWM1->PR = 12; // The TC is incremented every PR+1 cycles of PCLK.
    LPC_PWM1->LER |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 4) | (1 << 5); //Enable PWM Match 0+1+2+4+5 Latch
    LPC_PWM1->PCR |= (1 << 9) | (1 << 10) | (1 << 12) | (1 << 13); // PWMENA2
    LPC_PWM1->MCR |= (1 << 1); // PWMMR0R
    LPC_PWM1->TCR |= (1 << 0) | (1 << 3); // Counter Enable | PWM Enable

    GPIO_DIR_OUT(MOTOR_MOSFET);
    GPIO_CLR_PIN(MOTOR_MOSFET);

    GPIO_PIN_FNC(MOTOR_LEFT_PWM);
    GPIO_PIN_FNC(MOTOR_RIGHT_PWM);
    GPIO_PIN_FNC(MOTOR_BLADE_PWM);

    GPIO_DIR_OUT(MOTOR_LEFT_PWM);
    GPIO_DIR_OUT(MOTOR_RIGHT_PWM);
    GPIO_DIR_OUT(MOTOR_BLADE_PWM);

    GPIO_DIR_OUT(MOTOR_LEFT_ENABLE);
    GPIO_DIR_OUT(MOTOR_LEFT_BRAKE);
    GPIO_DIR_OUT(MOTOR_LEFT_FORWARD);
    
    GPIO_DIR_OUT(MOTOR_RIGHT_ENABLE);
    GPIO_DIR_OUT(MOTOR_RIGHT_BRAKE);
    GPIO_DIR_OUT(MOTOR_RIGHT_FORWARD);

    GPIO_DIR_OUT(MOTOR_BLADE_ENABLE);
    GPIO_DIR_OUT(MOTOR_BLADE_BRAKE);
    GPIO_DIR_OUT(MOTOR_BLADE_FORWARD);

// Set default modes ->  disabled controller, brake on, forward direction
    GPIO_CLR_PIN(MOTOR_LEFT_BRAKE);
    GPIO_CLR_PIN(MOTOR_RIGHT_BRAKE);
    GPIO_CLR_PIN(MOTOR_BLADE_BRAKE);

}