#include "LPC175x_6x_hal.h"



void hardware_Init() {
    LPC_SC->PCONP |= PCONP_PCGPIO;              // power up GPIO
    // Keep power on
    GPIO_DIR_OUT(POWER);
    GPIO_SET_PIN(POWER);

    // TODO: Turn on lcd backlight - Move to screen, and insert timer, and pwm.
    GPIO_DIR_OUT(LCD_BACKLIGHT);
    GPIO_SET_PIN(LCD_BACKLIGHT);

    // Configure Power button
    GPIO_PIN_FNC(KEYPAD_POWER);
    GPIO_FNC_PULL(KEYPAD_POWER, PINMODE_PULLDOWN);

}


void powerMgmt_Init() {
    // P1.21 - Charger connected (input, high active)
    // for approx. 2sec after connected to charger (led turns red), it is high

    //    GPIO_PINFNC(CHARGER_CONNECT); GPIO as std
    GPIO_FNC_PULL(CHARGER_CONNECTED, PINMODE_NEITHER);
    GPIO_DIR_IN(CHARGER_CONNECTED);

    // P1.23 - Enable charger (output, high active)
    //    GPIO_PINFNC(CHARGER_ENABLE); GPIO as std
    GPIO_FNC_PULL(CHARGER_ENABLE, PINMODE_NEITHER);
    GPIO_DIR_IN(CHARGER_ENABLE);
}

void sensor_Init() {
}
