#ifndef HAL_H
#define HAL_H

#include "LPC177x_8x.h"
#include "LPC177x_8x_hal.h"
#include "system_LPC177x_8x.h"
#include "common.h"

#define GPIO_TYPE(port, pin, pinsel)   ((gpioPin_t) { \
        ((LPC_GPIO_TypeDef *)(LPC_GPIO0_BASE + (0x20 * (port)))), \
        ((uint32_t *)(LPC_IOCON_BASE+(((port*32)+pin)* 4))), \
        ((uint8_t)(pin)), \
        (pinsel), \
        0 \
    })

#define PINMODE_NEITHER ((0))   /**< Inactive */
#define PINMODE_PULLDOWN ((1))  /**< Internal pull-down resistor */
#define PINMODE_PULLUP ((2))    /**< Internal pull-up resistor */
#define PINMODE_REPEAT ((3))    /**< Repeat mode */

#define PINMODE_NORMAL ((0))    /**< Pin is in the normal (not open drain) mode */
#define PINMODE_OPENDRAIN ((1)) /**< Pin is in the open drain mode */
#define PINMODE_INV ((1))       /**< Pin is inverted polarity */
#define PINMODE_FILTER ((1 << 8)) // Disable filtering

#define GPIO_PIN(gpio)     ((gpio.addr)->PIN)
#define GPIO_DIR(gpio)     ((gpio.addr)->DIR)
#define GPIO_BIT(gpio)     ((uint32_t)(1 << gpio.pin))
#define GPIO_PINNR(gpio)   ((uint32_t)gpio.pin)

#define GPIO_DIR_OUT(gpio) ((GPIO_DIR(gpio)) |=  (GPIO_BIT(gpio)))
#define GPIO_DIR_IN(gpio)  ((GPIO_DIR(gpio)) &= ~(GPIO_BIT(gpio)))
#define GPIO_GET_DIR(gpio) ((GPIO_DIR(gpio)) & (GPIO_BIT(gpio)))


//#define GPIO_PINMODE(gpio) (*(gpio.con) = (uint32_t)((*gpio.con & ~((3 << 3) | (1 << 10) | (1 << 6))) | ((gpio.pinmode << 3) | (gpio.od << 10) | (gpio.inv << 6))))

#define GPIO_PIN_FNC(gpio)  	 (*(gpio.con) = (uint32_t)((*gpio.con & ~(3 << 0)) | (gpio.pinsel << 0)))
#define GPIO_FNC_PULL(gpio, val) (*(gpio.con) = (uint32_t)((*gpio.con & ~(3 << 3)) | (val << 3)))
#define GPIO_FNC_INV(gpio, val)  (*(gpio.con) = (uint32_t)((*gpio.con & ~(1 << 6)) | (val << 6)))
#define GPIO_FNC_OD(gpio, val)   (*(gpio.con) = (uint32_t)((*gpio.con & ~(1 << 10)) | (val << 10)))

#define POWER              (GPIO_TYPE(PORT_3, PIN_6,  FUNC_0))
#define CHARGER_CHECK      (GPIO_TYPE(PORT_1, PIN_2, FUNC_0))
#define CHARGER_CONNECTED  (GPIO_TYPE(PORT_2, PIN_13, FUNC_0)) // If 1.2 High -> 2.13 Instantly low when connected to charger
//                                                               When Charge station turns RED -> 2.13 high, 3.13 High
#define CHARGER_ENABLE     (GPIO_TYPE(PORT_3, PIN_15, FUNC_0)) // Must enable charge within a few seconds.

#define MOTOR_MOSFET       (GPIO_TYPE(PORT_3, PIN_7,  FUNC_0)) // Does DB275 have this too?? 

#define BUZZER_HI          (GPIO_TYPE(PORT_2, PIN_30,  FUNC_0)) // T3_MAT2 <- to make different noices?
#define BUZZER_LO          (GPIO_TYPE(PORT_5, PIN_1,  FUNC_0))  // T2_MAT3 <- to make different noices?

#define MOTOR_LEFT_PWM       (GPIO_TYPE(PORT_3, PIN_27, FUNC_2))
#define MOTOR_LEFT_ENABLE    (GPIO_TYPE(PORT_1, PIN_17, FUNC_0))
#define MOTOR_LEFT_BRAKE     (GPIO_TYPE(PORT_4, PIN_24, FUNC_0))
#define MOTOR_LEFT_FORWARD   (GPIO_TYPE(PORT_4, PIN_25, FUNC_0)) // Eller reverse!
#define MOTOR_LEFT_PULSE     (GPIO_TYPE(PORT_1, PIN_14, FUNC_0))
#define MOTOR_LEFT_FAULT     (GPIO_TYPE(PORT_1, PIN_10, FUNC_0))

#define MOTOR_RIGHT_PWM      (GPIO_TYPE(PORT_3, PIN_28, FUNC_2))
#define MOTOR_RIGHT_ENABLE   (GPIO_TYPE(PORT_3, PIN_10, FUNC_0))
#define MOTOR_RIGHT_BRAKE    (GPIO_TYPE(PORT_3, PIN_2,  FUNC_0))
#define MOTOR_RIGHT_FORWARD  (GPIO_TYPE(PORT_5, PIN_4,  FUNC_0)) // Eller reverse!
#define MOTOR_RIGHT_PULSE    (GPIO_TYPE(PORT_3, PIN_30, FUNC_0))
#define MOTOR_RIGHT_FAULT    (GPIO_TYPE(PORT_0, PIN_3,  FUNC_0))

#define MOTOR_BLADE_PWM      (GPIO_TYPE(PORT_2, PIN_0,  FUNC_1))
#define MOTOR_BLADE_ENABLE   (GPIO_TYPE(PORT_4, PIN_28, FUNC_0))
#define MOTOR_BLADE_BRAKE    (GPIO_TYPE(PORT_4, PIN_29, FUNC_0))
#define MOTOR_BLADE_FORWARD  (GPIO_TYPE(PORT_1, PIN_6,  FUNC_0)) // Eller reverse!
#define MOTOR_BLADE_PULSE    (GPIO_TYPE(PORT_0, PIN_4,  FUNC_0))
#define MOTOR_BLADE_FAULT    (GPIO_TYPE(PORT_0, PIN_7,  FUNC_0))

#define LCD_BACKLIGHT      (GPIO_TYPE(PORT_2, PIN_1,  FUNC_0))
#define LCD_BACKLIGHT_PWM  (GPIO_TYPE(PORT_2, PIN_1,  FUNC_1))
#define LCD_A0             (GPIO_TYPE(PORT_0, PIN_9,  FUNC_0))
#define LCD_RSTB           (GPIO_TYPE(PORT_1, PIN_12, FUNC_0))
#define LCD_SDA            (GPIO_TYPE(PORT_0, PIN_8,  FUNC_0))
#define LCD_CSB            (GPIO_TYPE(PORT_4, PIN_13, FUNC_0))
#define LCD_SCLK           (GPIO_TYPE(PORT_4, PIN_14, FUNC_0))
#define LCD_CONTRAST	   55

#define KEYPAD_COL0        (GPIO_TYPE(PORT_0, PIN_20, FUNC_0))
#define KEYPAD_COL1        (GPIO_TYPE(PORT_4, PIN_26, FUNC_0))
#define KEYPAD_COL2        (GPIO_TYPE(PORT_0, PIN_21, FUNC_0))
#define KEYPAD_COL3        (GPIO_TYPE(PORT_0, PIN_22, FUNC_0))
#define KEYPAD_ROW0        (GPIO_TYPE(PORT_0, PIN_18, FUNC_0))
#define KEYPAD_ROW1        (GPIO_TYPE(PORT_4, PIN_22, FUNC_0))
#define KEYPAD_ROW2        (GPIO_TYPE(PORT_0, PIN_19, FUNC_0))
#define KEYPAD_ROW3        (GPIO_TYPE(PORT_4, PIN_7,  FUNC_0))

#define KEYPAD_STOP1       (GPIO_TYPE(PORT_1, PIN_25, FUNC_0))
#define KEYPAD_STOP2       (GPIO_TYPE(PORT_1, PIN_26, FUNC_0))
#define KEYPAD_POWER       (GPIO_TYPE(PORT_3, PIN_14, FUNC_0)) // PULL DOWN

#define SSP0_SCK           (GPIO_TYPE(PORT_1, PIN_20, FUNC_5))
#define SSP0_SSEL          (GPIO_TYPE(PORT_1, PIN_21, FUNC_3))
#define SSP0_MISO          (GPIO_TYPE(PORT_1, PIN_23, FUNC_5))
#define SSP0_MOSI          (GPIO_TYPE(PORT_1, PIN_24, FUNC_5))

#define SENSORS_SDA        (GPIO_TYPE(PORT_2, PIN_14, FUNC_2))
#define SENSORS_SDL        (GPIO_TYPE(PORT_2, PIN_15, FUNC_2))

#define SENSOR_STUCK       (GPIO_TYPE(PORT_2, PIN_22, FUNC_0))
#define SENSOR_STUCK2      (GPIO_TYPE(PORT_1, PIN_22, FUNC_0))
#define SENSOR_DOOR        (GPIO_TYPE(PORT_4, PIN_2, FUNC_0))
#define SENSOR_LIFT        (GPIO_TYPE(PORT_2, PIN_16, FUNC_0)) // or Collision
#define SENSOR_COLLISION   (GPIO_TYPE(PORT_4, PIN_1, FUNC_0)) // Or Lift!
#define SENSOR_STOP        (GPIO_TYPE(PORT_1, PIN_25, FUNC_0))
#define SENSOR_DOOR2       (GPIO_TYPE(PORT_2, PIN_21, FUNC_0))
#define SENSOR_RAIN        (GPIO_TYPE(PORT_1, PIN_30, FUNC_0)) // ADC!

#define SENSOR_BATT_BH     (GPIO_TYPE(PORT_5, PIN_0, FUNC_0))
#define SENSOR_BATT_BS     (GPIO_TYPE(PORT_3, PIN_13, FUNC_0))

#define ADC_DR_RESULT(n)        ((((n) >> 4) & 0xFFF))
// Note error in UM10470.pdf - filter should not affect ADC - But it does.
#define ADC_AD0				(GPIO_TYPE(PORT_0, PIN_23, FUNC_1 | PINMODE_FILTER))
#define ADC_AD1				(GPIO_TYPE(PORT_0, PIN_24, FUNC_1 | PINMODE_FILTER))
#define ADC_AD2				(GPIO_TYPE(PORT_0, PIN_25, FUNC_1 | PINMODE_FILTER))
#define ADC_AD3				(GPIO_TYPE(PORT_0, PIN_26, FUNC_1 | PINMODE_FILTER))
#define ADC_AD4				(GPIO_TYPE(PORT_1, PIN_30, FUNC_3 | PINMODE_FILTER))
#define ADC_AD5				(GPIO_TYPE(PORT_1, PIN_31, FUNC_3 | PINMODE_FILTER))
#define ADC_AD6				(GPIO_TYPE(PORT_0, PIN_12, FUNC_3 | PINMODE_FILTER))
#define ADC_AD7				(GPIO_TYPE(PORT_0, PIN_13, FUNC_3 | PINMODE_FILTER))

#define ANALOG_BATT_TEMP     (LPC_ADC->DR[0])
#define ANALOG_MOTOR_R_AMP   (LPC_ADC->DR[1])
#define ANALOG_MOTOR_L_AMP   (LPC_ADC->DR[2])
#define ANALOG_MOTOR_S_AMP   (LPC_ADC->DR[3])
#define ANALOG_RAIN          (LPC_ADC->DR[4])
#define ANALOG_BATT_CHARGE_A (LPC_ADC->DR[5])
#define ANALOG_BATT_VOLT     (LPC_ADC->DR[6])
#define ANALOG_BOARD_TEMP    (LPC_ADC->DR[7])

void hardware_Init();

void sensor_Init();
void powerMgmt_Init();
void MotorCtrl_Init();
void ROScomms_Init();

#endif
