#ifndef HAL_H
#define HAL_H

#include "LPC175x_6x.h"
#include "LPC175x_6x_hal.h"
#include "system_LPC175x_6x.h"
#include "common.h"

#define GPIO_TYPE(port, pin, pinsel, inv)   ((gpioPin_t) { \
        ((LPC_GPIO_TypeDef *)(LPC_GPIO_BASE + (0x20 * (port)))), \
        ((uint32_t *)LPC_PINCON_BASE + (port * 2) + (pin>15) ), \
        ((uint8_t)(pin)), \
        (pinsel), \
        (inv) \
    })

#define PINMODE_PULLUP      ((0))  /**< Internal pull-up resistor */
#define PINMODE_REPEAT      ((1))  /**< Repeat mode */
#define PINMODE_NEITHER     ((2))  /**< Tri-state / NEITHER */
#define PINMODE_PULLDOWN    ((3))  /**< Internal pull-down resistor */

#define NORM                ((0))
#define INV                 ((1))

#define PINMODE_NORMAL      ((0)) /**< Pin is in the normal (not open drain) mode */
#define PINMODE_OPENDRAIN   ((1)) /**< Pin is in the open drain mode */
#define PINMODE_INV         ((1)) /**< Pin is inverted polarity */

#define GPIO_PIN(gpio)     ((gpio.addr)->PIN)
#define GPIO_DIR(gpio)     ((gpio.addr)->DIR)
#define GPIO_BIT(gpio)     ((uint32_t)(1 << gpio.pin))
#define GPIO_PINNR(gpio)   ((uint32_t)gpio.pin)

#define GPIO_DIR_OUT(gpio) ((GPIO_DIR(gpio)) |=  (GPIO_BIT(gpio)))
#define GPIO_DIR_IN(gpio)  ((GPIO_DIR(gpio)) &= ~(GPIO_BIT(gpio)))
#define GPIO_GET_DIR(gpio) ((GPIO_DIR(gpio)) & (GPIO_BIT(gpio)))

#define SHIFTMODE(pin)          (2 * (pin - ((pin>15)?16:0))) //?
//#define GPIO_PINFNC(gpio)  (*(gpio.con) |= (uint32_t)(gpio.pinsel << (2 * (gpio.pin - ((gpio.pin>15)?16:0))) ))
// Dubbelkolla denna med! oven med if, nedan inte ? kolla manualus
#define GPIO_PIN_FNC(gpio)           (*(gpio.con)  = (uint32_t)((*(gpio.con) & ~(3 << 0)) | (gpio.pinsel << 0)))
#define GPIO_FNC_PULL(gpio, val)    (*(gpio.con+16) = (uint32_t)((*(gpio.con+16) & ~(3 << ( SHIFTMODE(gpio.pin) ))) \
                                                        | (val << ( SHIFTMODE(gpio.pin) ) )))

#define GPIO_FNC_OD(gpio, val)       (*(gpio.con+25) = (uint32_t)((*(gpio.con+25) & ~(1 << 10)) | (val << 10)))
#define test_get_con(gpio)           (*(gpio.con+16))
#define test_get_od(gpio)            (*(gpio.con+25))

/* Chatgpt:
#define GPIO_PIN_FNC(gpio)   (*(gpio.con) = (*gpio.con & ~(3 << ((gpio.pin & 0xF) * 2))) | (gpio.pinsel << ((gpio.pin & 0xF) * 2)))
#define GPIO_FNC_PULL(gpio, val) (*(gpio.con) = (*gpio.con & ~(3 << (((gpio.pin & 0xF) * 2) + 3))) | (val << (((gpio.pin & 0xF) * 2) + 3)))
#define GPIO_FNC_INV(gpio, val)  (*(gpio.con) = (*gpio.con & ~(1 << (((gpio.pin & 0xF) * 2) + 6))) | (val << (((gpio.pin & 0xF) * 2) + 6)))
#define GPIO_FNC_OD(gpio, val)   (*(gpio.con) = (*gpio.con & ~(1 << (((gpio.pin & 0xF) * 2) + 10))) | (val << (((gpio.pin & 0xF) * 2) + 10)))
*/

//FEL FEL FEL verkar bara sätta bitar men inte 0:or när man ändrar tillbaka! Måste nolla först!
//#define GPIO_FNC_PULL(gpio, val)    (*(gpio.con+16) = (uint32_t)(*(gpio.con+16) & ~(val << ( (gpio.pin > 15)?((gpio.pin*2)-32):(gpio.pin*2) ) )))
//#define GPIO_FNC_PULL(gpio, val)    (*(gpio.con+16) = (uint32_t)((*(gpio.con+16) & ~(3 << 0 )) | (val << 0 )))
//#define GPIO_FNC_PULL(gpio, val)    (*(gpio.con+16) = (uint32_t)( \

// inte ens denna fungerar ?!??!?

//                                                (*(gpio.con+16) & ~(3 << 3)) \
//                                                                 | (val << 3 )  \
//                                                                ))
//varför?
// 0x03000000 0x00000003 ???????
                                                                         

//#define GPIO_FNC_PULL(gpio, val)    (*(gpio.con+16) = (uint32_t)((*(gpio.con+16) & ~(3 << ( (gpio.pin > 15)?((gpio.pin*2)-32):(gpio.pin*2) ))) \
//                                                                         | (val << ( (gpio.pin > 15)?((gpio.pin*2)-32):(gpio.pin*2) ) )))
///// 1788 -> #define GPIO_FNC_PULL(gpio, val) (*(gpio.con) = (uint32_t)((*gpio.con & ~(3 << 3)) | (val << 3))) nolla först och sätt sen
//#define SPCR_BITS(n) ((n==0) ? ((uint32_t)0) : ((uint32_t)((n&0x0F)<<8)))
//#define GPIO_FNC_PULL(gpio, val) (*(gpio.con+16) = (uint32_t)(*((gpio.con+16) & ~(0b11 << 0)) | (val << 0 )))
//#define GPIO_FNC_PULL(gpio, val) (*(gpio.con+16) = (uint32_t)(*((gpio.con+16) & ~(0b11 << SHIFTMODE(gpio))) | (val << SHIFTMODE(gpio) )))

#define GPIO_FNC_INV(gpio, val)  // warning( - LPC1768 does not support port pin mode inverted polarity, change in aprotiate _HAL.h file instead!)

#define POWER              (GPIO_TYPE(PORT_1, PIN_25, FUNC_0, 0))
#define CHARGER_CONNECTED  (GPIO_TYPE(PORT_1, PIN_21, FUNC_0, 0)) // Is charger connected? high about 2s after connected to charger
#define CHARGER_ENABLE     (GPIO_TYPE(PORT_1, PIN_23, FUNC_0, 0)) // Enable charging
// TODO Check the following!
#define CHARGER_INIT       (GPIO_TYPE(PORT_0, PIN_11, FUNC_0, 0)) // IS this MOSFET power to motor controller??

#define LCD_BACKLIGHT      (GPIO_TYPE(PORT_1, PIN_20, FUNC_0, 0))
#define LCD_A0             (GPIO_TYPE(PORT_0, PIN_20, FUNC_0, 0))
#define LCD_RSTB           (GPIO_TYPE(PORT_0, PIN_19, FUNC_0, 0))
#define LCD_SDA            (GPIO_TYPE(PORT_0, PIN_18, FUNC_3, 0))
#define LCD_CSB            (GPIO_TYPE(PORT_0, PIN_16, FUNC_0, 0))
#define LCD_SCLK           (GPIO_TYPE(PORT_0, PIN_15, FUNC_3, 0))
#define LCD_CONTRAST	   3

#define KEYPAD_COL0        (GPIO_TYPE(PORT_1, PIN_0, FUNC_0, INV))
#define KEYPAD_COL1        (GPIO_TYPE(PORT_1, PIN_1, FUNC_0, INV))
#define KEYPAD_COL2        (GPIO_TYPE(PORT_1, PIN_4, FUNC_0, INV))
#define KEYPAD_COL3        (GPIO_TYPE(PORT_1, PIN_8, FUNC_0, INV))
#define KEYPAD_ROW0        (GPIO_TYPE(PORT_1, PIN_9, FUNC_0, INV))
#define KEYPAD_ROW1        (GPIO_TYPE(PORT_1, PIN_10, FUNC_0, INV))
#define KEYPAD_ROW2        (GPIO_TYPE(PORT_1, PIN_14, FUNC_0, INV))
#define KEYPAD_ROW3        (GPIO_TYPE(PORT_1, PIN_15, FUNC_0, INV))

#define KEYPAD_STOP1       (GPIO_TYPE(PORT_1, PIN_17, FUNC_0, 0))
#define KEYPAD_STOP2       (GPIO_TYPE(PORT_1, PIN_17, FUNC_0, 0)) // Only one!
#define KEYPAD_POWER       (GPIO_TYPE(PORT_1, PIN_28, FUNC_0, INV))

//#define CR 					ADCR
//#define PCLKSEL				PCLKSEL0
//#define INTEN				ADINTEN
//#define ADC_READ(val)		(( (uint32_t) ( (uint32_t *)(LPC_ADC_BASE + (10 + (val*4))) ) >> 4) & 4095)
#define ADC_DR_RESULT(n)        ((((n) >> 4) & 0xFFF))

#define ADC_AD0				(GPIO_TYPE(PORT_0, PIN_23, FUNC_1, 0))
#define ADC_AD1				(GPIO_TYPE(PORT_0, PIN_24, FUNC_1, 0))
#define ADC_AD2				(GPIO_TYPE(PORT_0, PIN_25, FUNC_1, 0))
#define ADC_AD3				(GPIO_TYPE(PORT_0, PIN_26, FUNC_1, 0))
#define ADC_AD4				(GPIO_TYPE(PORT_1, PIN_30, FUNC_3, 0))
#define ADC_AD5				(GPIO_TYPE(PORT_1, PIN_31, FUNC_3, 0))
#define ADC_AD6				(GPIO_TYPE(PORT_0, PIN_3, FUNC_2, 0))
#define ADC_AD7				(GPIO_TYPE(PORT_0, PIN_2, FUNC_2, 0))

// TODO FIX FIX FIX

#define ANALOG_BATT_CHARGE_A (LPC_ADC->DR[0])
#define ANALOG_BATT_VOLT     (LPC_ADC->DR[1])

// 2 = Tilt Sideways
// 3 = Tilt Forwards
// 4 = Muxing port P0.4+p0.5 :
//      0: X 
//      1: Y
//      2: Z
//      3: Batt Temp

#define ANALOG_BATT_TEMP     (0) // muxed! adc0.4

#define ANALOG_MOTOR_S_AMP   (LPC_ADC->DR[5])
#define ANALOG_MOTOR_L_AMP   (LPC_ADC->DR[6])
#define ANALOG_MOTOR_R_AMP   (LPC_ADC->DR[7])

#define ANALOG_RAIN          (LPC_ADC->DR[0])




void init_Hardware();

void sensor_Init();

void enable_Charger_Check();
void disable_Charger_Check();
void init_PowerMgmt();
/*
#define GPIO_TYPE(port, pin, pinsel, pinmode)   ((gpioPin_t) { \
    ((LPC_GPIO_TypeDef *)(LPC_GPIO0_BASE + (0x20 * (port)))), \
    ((uint32_t *)LPC_IOCON_BASE + (port * 2) + (pin>15) ), \
    ((uint8_t)(pin)), \
    (pinsel), \
    (pinmode) \
})
// STOP
#define KEYPAD_COL0_INV    (GPIO_TYPE(PORT_1, PIN_0, FUNC_0, PINMODE_PULLDOWN))
#define KEYPAD_COL1_INV    (GPIO_TYPE(PORT_1, PIN_1, FUNC_0, PINMODE_PULLDOWN))
#define KEYPAD_COL2_INV    (GPIO_TYPE(PORT_1, PIN_4, FUNC_0, PINMODE_PULLDOWN))
#define KEYPAD_COL3_INV    (GPIO_TYPE(PORT_1, PIN_8, FUNC_0, PINMODE_PULLDOWN))
#define KEYPAD_ROW0_INV    (GPIO_TYPE(PORT_1, PIN_9, FUNC_0, PINMODE_PULLUP))
#define KEYPAD_ROW1_INV    (GPIO_TYPE(PORT_1, PIN_10, FUNC_0, PINMODE_PULLUP))
#define KEYPAD_ROW2_INV    (GPIO_TYPE(PORT_1, PIN_14, FUNC_0, PINMODE_PULLUP))
#define KEYPAD_ROW3_INV    (GPIO_TYPE(PORT_1, PIN_15, FUNC_0, PINMODE_PULLUP))
*/

#endif

