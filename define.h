#ifndef DEFINES_H
#define DEFINES_H

#include <stdint.h>
#include "LPC17xx.h"


#define DO_PRAGMA(x) _Pragma (#x)
#define TODO(x) DO_PRAGMA(message ("TODO - " #x))
//Usage : TODO("stuff")

/*
#if __GNUC__
    TODO("Using GNUC compiler")
#endif

#if __KEIL__
    TODO("Using KEIL compiler")
#endif
*/

#define mainQUEUE_LENGTH          ( 10 )

#define TicksPerMS configTICK_RATE_HZ/1000

#define PCONP_PCTIM1    ((uint32_t)(1<<2))
#define PCONP_PCTIM2    ((uint32_t)(1<<22))
#define PCONP_PCUART1   ((uint32_t)(1<<4))
#define PCONP_PCSPI     ((uint32_t)(1<<8))
#define PCONP_PCADC     ((uint32_t)(1<<12))
#define PCONP_PCGPIO    ((uint32_t)(1<<15))

#define SPCR_MSTR       ((uint32_t)(1<<5))

#define PCLK_TIMER1(n)  ((uint32_t)(n<<4))
#define PCLK_TIMER2(n)  ((uint32_t)(n<<12))
#define PCLK_SPI(n)     ((uint32_t)(n<<16))
#define PCLK_ADC(n)     ((uint32_t)(n<<24))

#define CCLK_DIV4  ((uint32_t)(0))
#define CCLK_DIV1  ((uint32_t)(1))
#define CCLK_DIV2  ((uint32_t)(2))
#define CCLK_DIV8  ((uint32_t)(3))

#define PORT_0 ((0)) /**< PORT 0*/
#define PORT_1 ((1)) /**< PORT 1*/
#define PORT_2 ((2)) /**< PORT 2*/
#define PORT_3 ((3)) /**< PORT 3*/
#define PORT_4 ((4)) /**< PORT 4*/

#define FUNC_0 ((0)) /**< default function */
#define FUNC_1 ((1)) /**< first alternate function */
#define FUNC_2 ((2)) /**< second alternate function */
#define FUNC_3 ((3)) /**< third or reserved alternate function */

#define PINMODE_PULLUP ((0)) /**< Internal pull-up resistor */
#define PINMODE_TRISTATE ((2)) /**< Tri-state */
#define PINMODE_PULLDOWN ((3)) /**< Internal pull-down resistor */
#define PINMODE_NORMAL ((0)) /**< Pin is in the normal (not open drain) mode */
#define PINMODE_OPENDRAIN ((1)) /**< Pin is in the open drain mode */


#define PIN(n) ((uint32_t)(1 << (n)))
#define BIT(n) ((uint32_t)(1 << (n)))



typedef struct gpioPin_s {
    LPC_GPIO_TypeDef * addr;
    uint8_t bit;
} gpioPin_t;

#define GPIO_TYPE(addr, pin)   ((gpioPin_t) { ((LPC_GPIO_TypeDef *)(addr)), ((uint8_t)(pin)) })

#define GPIO_PIN(gpio)      ((gpio.addr)->FIOPIN)
#define GPIO_DIR(gpio)      ((gpio.addr)->FIODIR)
#define GPIO_BIT(gpio)      ((uint32_t)(1 << gpio.bit))
#define GPIO_PINNR(gpio)    ((uint32_t)gpio.bit)

#define GPIO_DIR_OUT(gpio)       ((GPIO_DIR(gpio)) |= (GPIO_BIT(gpio)))
#define GPIO_DIR_IN(gpio)        ((GPIO_DIR(gpio)) &= ~(GPIO_BIT(gpio)))

#define GPIO_SET_PIN(gpio)       ((GPIO_PIN(gpio)) |= (GPIO_BIT(gpio)))
#define GPIO_CLR_PIN(gpio)       ((GPIO_PIN(gpio)) &= ~(GPIO_BIT(gpio)))
#define GPIO_TGL_PIN(gpio)       ((GPIO_PIN(gpio)) ^= (GPIO_BIT(gpio)))
#define GPIO_CHK_PIN(gpio)       ((GPIO_PIN(gpio)) & (GPIO_BIT(gpio)))

#define GPIO_SET_PIN_VAL(gpio,val)   ( (GPIO_PIN(gpio)) ^= (-val ^ (GPIO_PIN(gpio)) ) & (GPIO_BIT(gpio)))

#define LCD_BACKLIGHT      (GPIO_TYPE(LPC_GPIO1, 20U))

#define LCD_A0             (GPIO_TYPE(LPC_GPIO0, 20U))
#define LCD_RSTB           (GPIO_TYPE(LPC_GPIO0, 19U))
#define LCD_SDA            (GPIO_TYPE(LPC_GPIO0, 18U))
#define LCD_CSB            (GPIO_TYPE(LPC_GPIO0, 16U))
#define LCD_SCLK           (GPIO_TYPE(LPC_GPIO0, 15U))


/* Advanced define with if...
#define DATABIT_15 SPI_SPCR_BITS(0x0F)
#define SPCR_BITS(n) ((n==0) ? ((uint32_t)0) : ((uint32_t)((n&0x0F)<<8)))

#pragma message ("Warning goes here")

#if (SETTING_A == 0) && (SETTING_B == 0)
#error SETTING_A and SETTING_B can't both be 0!
#endif

*/

#define KEY0        0
#define KEY1        1
#define KEY2        2
#define KEY3        3
#define KEY4        4
#define KEY5        5
#define KEY6        6
#define KEY7        7
#define KEY8        8
#define KEY9        9
#define KEYHOME     10
#define KEYUP       22
#define KEYDOWN     21
#define KEYBACK     91
#define KEYOK       90
#define KEYSTART    99
#define KEYPWR      100
#define KEYSTOP     199
#define KEYNULL     255


typedef enum {
    MEASUREMENT_BATTERY = 0,
    MEASUREMENT_MOTORCURRENT,
    COMMAND_SHUTDOWN,
    COMMAND_STOP,
} xMessageType;

#if (0)
typedef enum {
    CHARGE_CURRENT = 0,
    BATTERY_VOLTAGE,
    ACCELERATION_X,
    ACCELERATION_Y,
    ACCELERATION_Z,
    BATTERY_TEMPERATURE,
    SPINDLE_CURRENT,
    MOTOR_RIGHT_CURRENT,
    MOTOR_LEFT_CURRENT,
} SensorSource;

typedef enum {
    CURRENT = 0, /* in mA (unsigned int) */
    VOLTAGE, /* in mV (unsigned int) */
    ACCELERATION, /* in g (double) */
    TEMPERATURE, /* in °C (double) */
} DataFormat;

#pragma anon_unions
typedef struct {
    xMessageType type;
    uint32_t length;
    SensorSource source;
    DataFormat format;
    union {
        int32_t value;
        double valueFloat;
    };
} MessageSensor;
#endif

#endif // DEFINES_H
