#include <stdio.h>
#include "powermgmt.h"
#include "timers.h"

#define xDelay100  ((TickType_t)100 / portTICK_PERIOD_MS)

#define IS_CHARGER_CONNECTED()  GPIO_CHK_PIN(CHARGER_CONNECTED)
#define IS_CHARGER_ENABLED()    GPIO_CHK_PIN(CHARGER_ENABLE)
#define IS_CHARGING()           (IS_CHARGER_CONNECTED() && IS_CHARGER_ENABLED())

#define ENABLE_CHARGER()        GPIO_SET_PIN(CHARGER_ENABLE)
#define DISABLE_CHARGER()       GPIO_CLR_PIN(CHARGER_ENABLE)
#define ENABLE_MOTOR_MOSFET()   GPIO_SET_PIN(MOTOR_MOSFET)
#define DISABLE_MOTOR_MOSFET()  GPIO_CLR_PIN(MOTOR_MOSFET)
#define POWER_OFF()             GPIO_CLR_PIN(POWER)

#define BATTERY_MAX_VOLTAGE  287
#define BATTERY_MIN_VOLTAGE  237

// CHARGER_INIT
xQueueHandle xPowerMgmtMsgQueue;

int32_t lChargeCurrent;
int32_t lBatteryVoltage;
int32_t lBatteryTemperature;

void powerMgmt_Task(void *pvParameters) {
    xPowerMgmtMsgQueue = xQueueCreate(20, sizeof(xPowerMgmtMsg));
    lChargeCurrent = INT32_MIN;
    lBatteryVoltage = INT32_MIN;
    lBatteryTemperature = INT32_MIN;

    powerMgmt_Init();

    DISABLE_CHARGER();

    for (;;) {
        xPowerMgmtMsg msg;
        if (xQueueReceive(xPowerMgmtMsgQueue, &msg, xDelay100) == pdTRUE) {
            switch (msg.xType) {
                case (MEASUREMENT_BATTERY): {
                    /* https://hackaday.io/project/6717-project-landlord/discussion-58892 */
                    if (msg.measurement.lChargeCurrent != INT32_MIN) {
                        lChargeCurrent = msg.measurement.lChargeCurrent;
                        printf("Charge I: %4lumA\r\n", lChargeCurrent);
                    }
                    if (msg.measurement.lBatteryVoltage != INT32_MIN) {
                        // orig fw: v16_batteryLowV = 237; - 23.7V (fixed point) - 3.386V per cell
                        // orig fw: v18_batteryFullCharge = 287; - 28.7V (fixed point) - 4.1V per cell
                        lBatteryVoltage = msg.measurement.lBatteryVoltage;
                        printf("Battery U: %2lu.%01luV\r\n", lBatteryVoltage / 10, lBatteryVoltage % 10);
                        if ((lBatteryVoltage < BATTERY_MIN_VOLTAGE) && !IS_CHARGER_CONNECTED()) { /* below 23.7V - switch off to protect battery! */
                            /* issue shutdown to self */
                            xPowerMgmtMsg msg;
                            msg.xType = COMMAND_SHUTDOWN;
                            msg.shutdown.xDelay = xDelay100;
                            printf("Requesting shutdown to protect battery!\r\n");
                            xQueueSend(xPowerMgmtMsgQueue, &msg, (TickType_t)0);
                        } else if (lBatteryVoltage > BATTERY_MAX_VOLTAGE) { /* over 28.7V - stop charging! */
                        }
                    }
                    if (msg.measurement.lBatteryTemperature != INT32_MIN) {
                        lBatteryTemperature = msg.measurement.lBatteryTemperature;
                        if (lBatteryTemperature > 400) { /* above 40.0�C - switch off to protect battery! */
                            /* issue shutdown to self */
                            xPowerMgmtMsg msg;
                            msg.xType = COMMAND_SHUTDOWN;
                            msg.shutdown.xDelay = xDelay100;
                            printf("Requesting shutdown to protect battery!\r\n");
                            xQueueSend(xPowerMgmtMsgQueue, &msg, (TickType_t)0);
                        }
                        if (lBatteryTemperature >= 0) {
                            printf("Battery T: %3lu.%01lu%cC\r\n", lBatteryTemperature / 10, lBatteryTemperature % 10, '\xB0');
                        } else {
                            lBatteryTemperature = ~lBatteryTemperature;
                            printf("Battery T: -%3lu.%01lu%cC\r\n", lBatteryTemperature / 10, lBatteryTemperature % 10, '\xB0');
                        }

                    }
                    break;
                }
                case (COMMAND_SHUTDOWN): {
                    printf("Shutting down!\r\n");
                    vTaskDelay(msg.shutdown.xDelay);
                    POWER_OFF();
                    __disable_irq(); for (;;);
                }
                default:
                    printf("Unknown msg\r\n");
            }
        }
        // on each loop (latest every 100ms), check if connected to charger

        enable_Charger_Check();
        if (IS_CHARGER_CONNECTED()) {
            if (!IS_CHARGER_ENABLED()) {
                if ((lBatteryVoltage > (BATTERY_MIN_VOLTAGE - 10)) && (lBatteryVoltage < (BATTERY_MAX_VOLTAGE - 10))) {
                    printf("Start charging\r\n");
                    ENABLE_CHARGER();
                }
            } else {
                if (lBatteryVoltage >= BATTERY_MAX_VOLTAGE) {
                    printf("Stop charging\r\n");
                    DISABLE_CHARGER();
                }
            }
        } else {
            if (IS_CHARGER_ENABLED()) {
                printf("Charger disconnected\r\n");
                DISABLE_CHARGER();
            }
        }
        disable_Charger_Check();
#if LOWSTACKWARNING
        int stack = uxTaskGetStackHighWaterMark(NULL);
        if (stack < 50) printf("Task powerMgmt_Task has %u words left in stack.\r\n", stack);
#endif
    }
}
