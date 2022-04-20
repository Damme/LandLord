#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#ifdef LPC177x_8x // DB504
    #include "LPC177x_8x.h"
    #include "LPC177x_8x_hal.h"
    #include "system_LPC177x_8x.h"
#else
    #include "LPC175x_6x.h"
    #include "LPC175x_6x_hal.h"
    #include "system_LPC175x_6x.h"
#endif

#define DO_PRAGMA(x) _Pragma (#x)
#define TODO(x) DO_PRAGMA(message ("TODO - " #x))

#define HALT __disable_irq(); for( ;; );

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

#define PIN_0    (0U)    // Pin 0 
#define PIN_1    (1U)    // Pin 1 
#define PIN_2    (2U)    // Pin 2 
#define PIN_3    (3U)    // Pin 3 
#define PIN_4    (4U)    // Pin 4 
#define PIN_5    (5U)    // Pin 5 
#define PIN_6    (6U)    // Pin 6 
#define PIN_7    (7U)    // Pin 7 
#define PIN_8    (8U)    // Pin 8 
#define PIN_9    (9U)    // Pin 9 
#define PIN_10   (10U)   // Pin 10
#define PIN_11   (11U)   // Pin 11
#define PIN_12   (12U)   // Pin 12
#define PIN_13   (13U)   // Pin 13
#define PIN_14   (14U)   // Pin 14
#define PIN_15   (15U)   // Pin 15
#define PIN_16   (16U)   // Pin 16
#define PIN_17   (17U)   // Pin 17
#define PIN_18   (18U)   // Pin 18
#define PIN_19   (19U)   // Pin 19
#define PIN_20   (20U)   // Pin 20
#define PIN_21   (21U)   // Pin 21
#define PIN_22   (22U)   // Pin 22
#define PIN_23   (23U)   // Pin 23
#define PIN_24   (24U)   // Pin 24
#define PIN_25   (25U)   // Pin 25
#define PIN_26   (26U)   // Pin 26
#define PIN_27   (27U)   // Pin 27
#define PIN_28   (28U)   // Pin 28
#define PIN_29   (29U)   // Pin 29
#define PIN_30   (30U)   // Pin 30
#define PIN_31   (31U)   // Pin 31

#define PORT_0   (0U) // PORT 0
#define PORT_1   (1U) // PORT 1
#define PORT_2   (2U) // PORT 2
#define PORT_3   (3U) // PORT 3
#define PORT_4   (4U) // PORT 4
#define PORT_5   (5U) // PORT 5 (Only LPC1788)

#define FUNC_0   (0U) // default function
#define FUNC_1   (1U) // first alternate function
#define FUNC_2   (2U) // second alternate function
#define FUNC_3   (3U) // third  alternate function
#define FUNC_4   (4U) // fourth alternate function (Only LPC1788)
#define FUNC_5   (5U) // fifth alternate function (Only LPC1788)
#define FUNC_6   (6U) // sixth alternate function (Only LPC1788)
#define FUNC_7   (7U) // seventh alternate function (Only LPC1788)


#define PIN(n) ((uint32_t)(1 << (n)))
#define BIT(n) ((uint32_t)(1 << (n)))


typedef struct gpioPin_s {
    LPC_GPIO_TypeDef * addr;
    uint32_t * con;
    uint8_t pin;
    uint8_t pinsel;
    uint8_t inv;
} gpioPin_t;

// TODO No "fake invert" on SET & CLR only CHK!

#define GPIO_SET_PIN(gpio)           ((GPIO_PIN(gpio)) |=  (GPIO_BIT(gpio)))
#define GPIO_SET_PIN_VAL(gpio,val)   ((GPIO_PIN(gpio)) ^=  (-val ^ (GPIO_PIN(gpio)) ) & (GPIO_BIT(gpio)))
#define GPIO_CLR_PIN(gpio)           ((GPIO_PIN(gpio)) &= ~(GPIO_BIT(gpio)))
#define GPIO_TGL_PIN(gpio)           ((GPIO_PIN(gpio)) ^=  (GPIO_BIT(gpio)))
#define GPIO_CHK_PIN(gpio)           (((GPIO_PIN(gpio) >> GPIO_PINNR(gpio)) & (gpio.inv?0:1)))


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
    TEMPERATURE, /* in �C (double) */
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
