#include <stdio.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "common.h"

#include "powermgmt.h"
#include "timers.h"
#include "global.h"
#include "keypad.h"
#include "ROSComms.h"

// Battery: 7S Li-ion (29.4 V max), the charger ends at about 29.1 V.
#define BATTERY_MAX_VOLT   29300 // charging above this -> finished
#define BATTERY_DELTA_VOLT 750   // finished: recharge below BATTERY_MAX_VOLT - this
#define BATTERY_FULL_VOLT  28800 // charge current stopped at/above this = the charger finished

// Charge temperature window, 0.1 degC (filtered in sensor.c), with hysteresis.
#define CHARGE_TEMP_HOT      430
#define CHARGE_TEMP_HOT_OK   400
#define CHARGE_TEMP_COLD     0
#define CHARGE_TEMP_COLD_OK  30

#define CHARGE_MIN_MA      20       // current that counts as charging
#define CHARGE_START_MS    15000    // after enabling: wait this long for current
#define CHARGE_LOST_MS     10000    // current gone this long -> charge ended
#define CHARGE_RETRY_MS    60000    // after a failure, while not moved: try again
#define CHARGE_RETRY_SLOW_MS 600000 // ... after CHARGE_RETRY_FAST failures
#define CHARGE_RETRY_FAST  3
#define MOVED_TICKS        200      // left+right pulses (~24 cm): left the dock

/*
7 x 18650
3.2 x 7 = 22.4v
3.4 x 7 = 23.8v
4.1 x 7 = 28.7v
4.2 x 7 = 29.4v
2sec until red
4sec red

8 x lifepo4:
2.3  x 8 = 18.4v MINMIN cutoff (2.50v release)
2.4  x 8 = 19.2v
2.5  x 8 = 20.0v MIN
2.6  x 8 = 20.8v
3.65 x 8 = 29.2v
3.7  x 8 = 29.6v MAX
3.75 x 8 = 30.0v MAXMAX cutoff (3.55v release)

Charging (robot logs 2026-09-29): the current tapers to ~160 mA at 29.07 V and
then stops without reaching BATTERY_MAX_VOLT. The old code called that
ChargingFailed and waited in CheckCharger for CHARGER_CONNECTED, which only
reads with MOTOR_MOSFET on: with the motors off (at night) it never charged
again. Now a charge that ends at >= BATTERY_FULL_VOLT is finished, and a failed
start is retried while the robot hasn't moved.
*/

const char *powerStateStrings[] = {
    "Startup",
    "Idle",
    "CheckChargerInit",
    "CheckCharger",
    "StartCharging",
    "Charging",
    "BatteryCooldown",
    "ChargingFinished",
    "ChargingFailed",
    "Shutdown",
    "BatteryCold"
};

static uint32_t wheelTicks(void) {
    return sensorMsg.motorPulseLeft + sensorMsg.motorPulseRight;
}

static bool elapsed(TickType_t since, uint32_t ms) {
    return (xTaskGetTickCount() - since) >= pdMS_TO_TICKS(ms);
}

// Charger keep-alive: a short ENABLE pulse keeps the station in its red
// (charging) state while we don't take current.
static void keepAlivePulse(void) {
    GPIO_SET_PIN(CHARGER_ENABLE);
    vTaskDelay(xDelay10); // 10ms is enough
    GPIO_CLR_PIN(CHARGER_ENABLE);
}

static bool tempOk(int32_t temp) {
    return temp <= CHARGE_TEMP_HOT && temp >= CHARGE_TEMP_COLD;
}

void powerMgmt_Task(void *pvParameters) {
    powerMgmt_Init();

    // pulse charger mosfet incase we already is in charger.    
    keepAlivePulse();
    vTaskDelay(xDelay2000); // battery filters settle (sensor task)
    
    // Boot probe: try to charge once. CHARGER_CONNECTED can't tell (it needs
    // MOTOR_MOSFET), and a CPU reset on the dock must not leave us not charging.
    TickType_t delay = xDelay100;
    powerStates powerState = StartCharging;
    powerStates lastState = -1;

    TickType_t stateSince = xTaskGetTickCount();
    TickType_t lastCurrentAt = 0;
    TickType_t retryAt = 0;
    bool retryPending = false;
    bool seenCurrent = false;
    int32_t chargeMv = 0;         // voltage while current last flowed
    uint32_t contactTicks = wheelTicks(); // wheel ticks when we last had contact evidence
    uint8_t failures = 0;
    
    for (;;) {
        if (powerState != lastState) {
            lastState = powerState;
            stateSince = xTaskGetTickCount();
            sensorMsg.powerState = powerState;
            xJSONMessageType JSONMsg = {"powerState", {0}};
            strcpy(JSONMsg.value, powerStateStrings[powerState]);
            xQueueSend(xJSONMessageQueue, &JSONMsg, xDelay100);
        }

        // Contact evidence: charge current. Without it, driving away clears InCharger.
        bool current = sensorMsg.batteryChargeCurrent >= CHARGE_MIN_MA;
        if (current) {
            contactTicks = wheelTicks();
            sensorMsg.inCharger = 1;
            failures = 0;
        } else if (sensorMsg.inCharger && wheelTicks() - contactTicks > MOVED_TICKS) {
            sensorMsg.inCharger = 0;
            retryPending = false;
            if (powerState != CheckCharger && powerState != CheckChargerInit && powerState != Shutdown) {
                GPIO_CLR_PIN(CHARGER_ENABLE);
                powerState = Idle;
            }
        }

        switch(powerState) {
            case Startup:
            case Idle:
                GPIO_CLR_PIN(CHARGER_ENABLE);
                sensorMsg.inCharger = 0;
                powerState = CheckChargerInit;
                break;

            case CheckChargerInit:
                delay = xDelay100;
                GPIO_SET_PIN(CHARGER_CHECK);
                powerState = CheckCharger;
                break;

            case CheckCharger:
                if (GPIO_CHK_PIN(CHARGER_CONNECTED) || current) {
                    sensorMsg.inCharger = 1;
                    contactTicks = wheelTicks();
                    powerState = StartCharging;
                } else if (retryPending && sensorMsg.inCharger && xTaskGetTickCount() - retryAt < 0x80000000UL) {
                    retryPending = false;
                    debug("Charging: retry %u", failures);
                    powerState = StartCharging;
                }
                break;

            case StartCharging:
                GPIO_CLR_PIN(CHARGER_CHECK);
                if (!tempOk(sensorMsg.batteryTemp)) {
                    powerState = sensorMsg.batteryTemp > CHARGE_TEMP_HOT ? BatteryCooldown : BatteryCold;
                    break;
                }
                GPIO_SET_PIN(CHARGER_ENABLE);
                delay = xDelay100;
                seenCurrent = false;
                powerState = Charging;
                break;

            case Charging:
                if (current) {
                    seenCurrent = true;
                    lastCurrentAt = xTaskGetTickCount();
                    chargeMv = sensorMsg.batteryVolt;
                    // TODO Count mAh/Wh charged
                }
                if (sensorMsg.batteryTemp > CHARGE_TEMP_HOT) {
                    GPIO_CLR_PIN(CHARGER_ENABLE);
                    powerState = BatteryCooldown;
                } else if (sensorMsg.batteryTemp < CHARGE_TEMP_COLD) {
                    GPIO_CLR_PIN(CHARGER_ENABLE);
                    powerState = BatteryCold;
                } else if (sensorMsg.batteryVolt > BATTERY_MAX_VOLT) {
                    GPIO_CLR_PIN(CHARGER_ENABLE);
                    powerState = ChargingFinished;
                } else if (seenCurrent && elapsed(lastCurrentAt, CHARGE_LOST_MS)) {
                    // The charger stopped: full, or contact/charger lost.
                    GPIO_CLR_PIN(CHARGER_ENABLE);
                    powerState = chargeMv >= BATTERY_FULL_VOLT ? ChargingFinished : ChargingFailed;
                } else if (!seenCurrent && elapsed(stateSince, CHARGE_START_MS)) {
                    GPIO_CLR_PIN(CHARGER_ENABLE);
                    powerState = ChargingFailed;
                }
                break;

            case BatteryCooldown:
            case BatteryCold:
                delay = xDelay1000;
                keepAlivePulse();
                if (powerState == BatteryCooldown ? sensorMsg.batteryTemp < CHARGE_TEMP_HOT_OK
                                                  : sensorMsg.batteryTemp > CHARGE_TEMP_COLD_OK) {
                    delay = xDelay100;
                    powerState = StartCharging;
                }
                break;

            case ChargingFinished:
                // TODO Signal ROS charge finished
                delay = xDelay100;
                vTaskDelay(xDelay500);
                keepAlivePulse();
                // TODO Seems analog charge current is peak value, so even if we only have the charge enabled for 0.025s per
                // second we still get a value of > 300mA. maybe we should "correct" this in software?
                if (sensorMsg.batteryVolt < ( BATTERY_MAX_VOLT - BATTERY_DELTA_VOLT) ) powerState = StartCharging;
                break;

            case ChargingFailed:
                // Contact bounce, a charger that isn't ready, mains loss...: try again
                // while the robot stays where it is (moving away clears InCharger).
                failures++;
                debug("Charging failed (%u), mV %li mA %li Temp %li Contact %u", failures, sensorMsg.batteryVolt,
                    sensorMsg.batteryChargeCurrent, sensorMsg.batteryTemp, GPIO_CHK_PIN(CHARGER_CONNECTED));
                retryAt = xTaskGetTickCount() + pdMS_TO_TICKS(failures <= CHARGE_RETRY_FAST ? CHARGE_RETRY_MS : CHARGE_RETRY_SLOW_MS);
                retryPending = true;
                powerState = CheckChargerInit;
                break;

            case Shutdown:
                vTaskDelay(xDelay250);
                GPIO_CLR_PIN(MOTOR_MOSFET);
                GPIO_CLR_PIN(CHARGER_ENABLE);
                GPIO_CLR_PIN(LCD_BACKLIGHT);
                GPIO_CLR_PIN(CHARGER_CHECK);
                GPIO_CLR_PIN(POWER);
                HALT;
                break;

        }

        // TODO Check too high temp -> shut down

        if (keypad_GetKey() == KEYPWR) {
            if (keypad_GetTime() > 15) {
                powerState = Shutdown;
            }
        }
        
        vTaskDelay(delay);
#if LOWSTACKWARNING
        int stack = uxTaskGetStackHighWaterMark(NULL);
        if (stack < 50) printf("Task powerMgmt_Task has %u words left in stack.\r\n", stack);
#endif
    }
}
