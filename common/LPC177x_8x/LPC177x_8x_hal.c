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
/*
    GPIO_FNC_PULL(CHARGER_CONNECTED, PINMODE_PULLUP);
    GPIO_DIR_IN(CHARGER_CONNECTED);

    GPIO_FNC_PULL(CHARGER_ENABLE, PINMODE_PULLUP);
    GPIO_DIR_IN(CHARGER_ENABLE);*/

//#define GPIO_BATT_BH        (GPIO_TYPE(PORT_3, PIN_13, FUNC_0))
//#define GPIO_BATT_BS        (GPIO_TYPE(PORT_5, PIN_0, FUNC_0))
    //GPIO_FNC_PULL(GPIO_BATT_BH, PINMODE_PULLUP);
    //GPIO_FNC_PULL(GPIO_BATT_BS, PINMODE_PULLUP);  
}

void sensor_Init() {
    uint32_t fullAdcRate;
    uint8_t div;
    //fullAdcRate = 400000 * 31;
    fullAdcRate = 375000 * 33;
    div = ((PeripheralClock * 2 + fullAdcRate) / (fullAdcRate * 2)) - 1;

    GPIO_PIN_FNC(ADC_AD0);
    GPIO_PIN_FNC(ADC_AD1);
    GPIO_PIN_FNC(ADC_AD2);
    GPIO_PIN_FNC(ADC_AD3);
    GPIO_PIN_FNC(ADC_AD4);
    GPIO_PIN_FNC(ADC_AD5);
    GPIO_PIN_FNC(ADC_AD6);
    GPIO_PIN_FNC(ADC_AD7);
    
    LPC_ADC->CR = (div << 8) | (1 << 21); //div 4 400000hz, enable ADC ändrat till div 8
    LPC_ADC->CR |= ( 0xff ); // enable all adc channels.
    LPC_ADC->CR |= (1UL<<16); // enable Burst mode
}

void powerMgmt_Init() {
    GPIO_DIR_OUT(CHARGER_CHECK);
    GPIO_DIR_OUT(CHARGER_ENABLE);
}

void enable_Charger_Check() {
    GPIO_SET_PIN(CHARGER_CHECK);
}

void disable_Charger_Check() {
    GPIO_CLR_PIN(CHARGER_CHECK);
}

void MotorCtrl_Init() {
// Configure PWM 1.1(Blade) 1.4(left) 1.5(right) (and pwm1.2 LCD brightness)
    LPC_PWM1->MR0 = 1000;
    LPC_PWM1->MR1 = 0;      // Blade
    LPC_PWM1->MR4 = 0;      // Left
    LPC_PWM1->MR5 = 0;      // Right
    LPC_PWM1->MR2 = 1000;   // LCD
// Setup PWM registers
    // mr0 = 1000 pr = 59 : ~1k hz
    // mr0 = 1000 pr = 12 : ~4.6 khz
    // mr0 = 1000 pr = 5  : ~10 khz
    // mr0 = 1000 pr = 2  : ~20 khz
    LPC_PWM1->PR = 12;
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
