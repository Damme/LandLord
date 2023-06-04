#include <stdio.h>
#include "powermgmt.h"
#include "timers.h"
#include "global.h"
#include "keypad.h"

#define BATTERY_MAX_TEMP  400
#define BATTERY_MAX_VOLT  29300
#define BATTERY_MIN_VOLT  22000
#define BATTERY_DELTA_VOLT 1500
#define BATTERY_MAX_AMP   4000

/*
7 x 18650
3.2 x 7 = 22.4v
3.4 x 7 = 23.8v
4.1 x 7 = 28.7v
4.2 x 7 = 29.4v
2sec until red
4sec red
*/

typedef enum {
    Startup,
    Idle,
    CheckChargerInit,
    CheckCharger,
    StartCharging,
    Charging,
    BatteryCooldown,
    ChargingFinished,
    ChargingFailed,
    Shutdown
} powerStates;

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
    "Shutdown"
};

void powerMgmt_Task(void *pvParameters) {
    powerMgmt_Init();

    // pulse charger mosfet incase we already is in charger.    
    GPIO_SET_PIN(CHARGER_ENABLE);
    vTaskDelay(xDelay10); // 10ms is enough
    GPIO_CLR_PIN(CHARGER_ENABLE);
    
    TickType_t delay = xDelay50;
    powerStates powerState = CheckChargerInit;
    powerStates lastState = 0;
    
    //xScreenMsgType screenMsg;
    uint16_t count = 5;
    
    for (;;) {
        if (powerState != lastState) {
            lastState = powerState;
            xJSONMessageType JSONMsg = {"powerState", {0}};
            strcpy(JSONMsg.value, powerStateStrings[powerState]);
            xQueueSend(xJSONMessageQueue, &JSONMsg, xDelay100);
        }
        switch(powerState) {
            case Startup:
                // if checking for charger all the time we dont need this forced charging function.
                if (keypad_GetKey() == KEYHOME) {
                    powerState = CheckChargerInit;
                    
                    //screenMsg.time=50;
                    //sprintf(screenMsg.text, "Forced charging");      
                    //xQueueSend(xScreenMsgQueue, &screenMsg, (TickType_t)0);
                }
                count--;
                if (count == 0) powerState = Idle;
                break;

            case Idle:
                // Todo dim display if idle > x minutes, idle is if lid is closed!
                delay = xDelay500;
                count = 500;
                powerState = CheckChargerInit;
                sensorMsg.inCharger = 0;
                break;

            case CheckChargerInit:
                // TODO only start charging if volt below xx?
                count = 100;
                delay = xDelay50;
                GPIO_SET_PIN(CHARGER_CHECK);
                powerState = CheckCharger;
                break;

            case CheckCharger:
                if (GPIO_CHK_PIN(CHARGER_CONNECTED) || sensorMsg.batteryChargeCurrent > 5) {
                    GPIO_CLR_PIN(CHARGER_CHECK);
                    sensorMsg.inCharger = 1;
                    powerState = StartCharging;

                    //screenMsg.time=15;
                    //sprintf(screenMsg.text, "In charger!");
                    //xQueueSend(xScreenMsgQueue, &screenMsg, (TickType_t)0);
                }
                /*
                // Need to rewrite this logic to handle state mowing, going to dock etc.
                count--;
                if (count < 1) {
                    GPIO_CLR_PIN(CHARGER_CHECK);
                    powerState = Idle;
                }*/
                break;
// Catch state connected to charger but charger not ready (For example, cpu reset with charger already connected but timed out)
// CheckCharger Yes -> StartCharging
// StartCharging -> Charging
// 4s elapses -> Charger disconnected! -> ChargingFailed 
// -> CheckChargerInit -> CheckChargerInit

            case StartCharging:
                GPIO_SET_PIN(CHARGER_ENABLE);
                delay = xDelay100;
                powerState = Charging;
                count = 50;
                break;

            case Charging:
                if (sensorMsg.batteryChargeCurrent > 5) {
                    count = 50;
                    // TODO Tempeature checks
                    if (sensorMsg.batteryTemp > 430) powerState = BatteryCooldown;
                    // TODO Count mAh/Wh charged
                    // TODO Start time and elapsed charging time
                    if (!sensorMsg.inCharger) {
                        sensorMsg.inCharger = 1;
                    }
                }

                if (sensorMsg.batteryVolt > BATTERY_MAX_VOLT) {
                    powerState = ChargingFinished;
                    
                    //screenMsg.time=15;
                    //sprintf(screenMsg.text, "Charge finished!");
                    //xQueueSend(xScreenMsgQueue, &screenMsg, (TickType_t)0);
                }

                count--;
                if (count == 0) {
                    powerState = ChargingFailed;
                    
                    //screenMsg.time=15;
                    //sprintf(screenMsg.text, "Charger disconnected!");
                    //xQueueSend(xScreenMsgQueue, &screenMsg, (TickType_t)0);
                }
                break;

            case BatteryCooldown:
                delay = xDelay1000;
                GPIO_SET_PIN(CHARGER_ENABLE);
                vTaskDelay(xDelay10); // 10ms is enough
                GPIO_CLR_PIN(CHARGER_ENABLE);
                if (sensorMsg.batteryTemp < 420) powerState = StartCharging;
                break;

            case ChargingFinished:
                // TODO Signal ROS charge finished
                delay = xDelay100;
                vTaskDelay(xDelay500);
                GPIO_SET_PIN(CHARGER_ENABLE);
                vTaskDelay(xDelay10); // 10ms is enough
                GPIO_CLR_PIN(CHARGER_ENABLE);
                // TODO Seems analog charge current is peak value, so even if we only have the charge enabled for 0.025s per
                // second we still get a value of > 300mA. maybe we should "correct" this in software?

                // check if we get ADC value to see if charger is still alive? (how long time charge period does this require?)
                if (!sensorMsg.inCharger) powerState = Idle;
                // TODO TODO Noone clears incharger state as of now, Either ROS comms when backing out of charger or sensor when 
                // forward motion is detected.
                
                if (sensorMsg.batteryVolt < ( BATTERY_MAX_VOLT - BATTERY_DELTA_VOLT) ) powerState = StartCharging;
                break;

            case ChargingFailed:
                delay = xDelay1000;
                // mains power loss ? retry start charging over and over again until success?
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
            //screenMsg.time=4;
            //sprintf(screenMsg.text, "Shutting down! %i", keypad_GetTime());
            //xQueueSend(xScreenMsgQueue, &screenMsg, (TickType_t)0);
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
