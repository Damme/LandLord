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


    GPIO_DIR_OUT(MOTOR_MOSFET);

/*
#define GPIO_TEST        (GPIO_TYPE(PORT_0, PIN_13, FUNC_0))
    //GPIO_FNC_PULL(GPIO_TEST, PINMODE_PULLDOWN);
    GPIO_DIR_OUT(GPIO_TEST);
    GPIO_SET_PIN(GPIO_TEST);
*/

//#define GPIO_BATT_BH        (GPIO_TYPE(PORT_3, PIN_13, FUNC_0))
//#define GPIO_BATT_BS        (GPIO_TYPE(PORT_5, PIN_0, FUNC_0))
    //GPIO_FNC_PULL(GPIO_BATT_BH, PINMODE_PULLUP);
    //GPIO_FNC_PULL(GPIO_BATT_BS, PINMODE_PULLUP);
    
    /*GPIO_FNC_PULL(GPIO_TEST, PINMODE_PULLDOWN);
    GPIO_DIR_OUT(GPIO_TEST);
    GPIO_SET_PIN(GPIO_TEST);
*/
    //GPIO_FNC_PULL(GPIO_TEST, PINMODE_PULLUP);   
}

void sensor_Init() {
    uint32_t fullAdcRate;
    uint8_t div;
    //fullAdcRate = 400000 * 31;
    fullAdcRate = 375000 * 33;
    div = ((PeripheralClock * 2 + fullAdcRate) / (fullAdcRate * 2)) - 1;

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
}

void enable_Charger_Check() {
    GPIO_SET_PIN(CHARGER_CHECK);
}

void disable_Charger_Check() {
    GPIO_CLR_PIN(CHARGER_CHECK);
}

