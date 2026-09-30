#include <stdio.h>
#include "motorctrl.h"
#include "common.h"
#include "global.h"
#include "timers.h"
#include "ROSComms.h"

int16_t setPWMBlade, setPWMLeft, setPWMRight = 0;
int16_t curPWMBlade, curPWMLeft, curPWMRight = 0;
int32_t tiltCounter = 0;
int32_t liftCounter = 0;
int32_t bumpOn = 0, bumpOff = 0;
bool bumpLatched = false;

#define BUMP_ON_MS  5  // pressed this long = one bump
#define BUMP_OFF_MS 50 // released this long before the next one counts

#define TILT_EMG_MS 500
#define LIFT_EMG_MS 100 // filters contact glitches at the charger (seen 2026-09-28)

void setpwm(uint16_t blade, uint16_t left, uint16_t right) {
    if (blade > 2047) blade=2047;
    if (left > 2047) left=2047;
    if (right > 2047) right=2047;

    if (blade < 0) blade=0;
    if (left < 0) left=0;
    if (right < 0) right=0;

    LPC_PWM1->MR1 = blade;
    LPC_PWM1->MR4 = left;
    LPC_PWM1->MR5 = right;
    LPC_PWM1->LER |= (1<<1) | (1<<4) | (1<<5);    
}

const char *MotorRequestStrings[] = {
    "MOTORREQ_IDLE",
    "MOTORREQ_ENABLE",
    "MOTORREQ_DISABLE",
    "MOTORREQ_SETSPEED",
    "MOTORREQ_BRAKEON",
    "MOTORREQ_BRAKEOFF",
    "MOTORREQ_EMGSTOP",
    "MOTORREQ_RESETEMG"
};


// Queues an emergency stop. Callable from the timer task: never blocks.
static void requestEmergencyFromISR(uint8_t reason, BaseType_t *woken) {
    xMotorMsgType MotorMsg = { .action = MOTORREQ_EMGSTOP, .reason = reason, .pwm = {0, 0, 0} };
    xQueueSendFromISR(xMotorMsgQueue, &MotorMsg, woken);
}

// Every 1 ms from TIMER2_IRQHandler (sensor.c), so the counts are real
// milliseconds. It used to be a 1 ms FreeRTOS software timer, but the timer task
// has priority 3, below most tasks: when it ran late, FreeRTOS called the
// missed callbacks back to back, all reading the pin at the same moment. A lift
// pin low for a moment (bump on 2026-09-30 18:41) then counted as 100 ms and
// latched a false lift emergency.
void motionSensor_TickFromISR(BaseType_t *woken) {
    if (!GPIO_CHK_PIN(SENSOR_STUCK) || !GPIO_CHK_PIN(SENSOR_STUCK2) || !GPIO_CHK_PIN(SENSOR_LIFT) || !GPIO_CHK_PIN(SENSOR_COLLISION))
        sensorMsg.blockForward = 1;

    // Bump counter: bumper pins (low = pressed; all three go low on the dock).
    // ROS sees every press, even one shorter than a report period.
    bool bump = !GPIO_CHK_PIN(SENSOR_STUCK) || !GPIO_CHK_PIN(SENSOR_STUCK2) || !GPIO_CHK_PIN(SENSOR_COLLISION);
    if (bump) {
        bumpOff = 0;
        if (!bumpLatched && ++bumpOn >= BUMP_ON_MS) {
            bumpLatched = true;
            sensorMsg.bumps++;
        }
    } else {
        bumpOn = 0;
        if (bumpLatched && ++bumpOff >= BUMP_OFF_MS) bumpLatched = false;
    }

    // If robot tilted too much / upside down and blade on emergancy stop!
    if (sensorMsg.accelZ < 5000 && sensorMsg.currentPWMBlade > 0) {
        if (++tiltCounter == TILT_EMG_MS) requestEmergencyFromISR(EMG_TILT, woken);
    } else {
        tiltCounter = 0;
    }

    // Lift (pin low = lifted) while the motors are on.
    if (sensorMsg.motorsEnabled && !GPIO_CHK_PIN(SENSOR_LIFT)) {
        if (++liftCounter == LIFT_EMG_MS) requestEmergencyFromISR(EMG_LIFT, woken);
    } else {
        liftCounter = 0;
    }
}
void allStop() {
    setpwm(0,0,0);
    setPWMBlade = setPWMLeft = setPWMRight = 0;
    curPWMBlade = curPWMLeft = curPWMRight = 0;
}

#define INCREASE_RATE 300
#define DECREASE_RATE 500
// More work to be done if deaccel should be faster than accel
void ramp(int16_t* target_speed, int16_t* current_speed) {
    if (*current_speed < *target_speed) {
        *current_speed += INCREASE_RATE;
        if (*current_speed > *target_speed) *current_speed = *target_speed;
    }
    if (*current_speed > *target_speed) {
        *current_speed -= INCREASE_RATE;
        if (*current_speed < *target_speed) *current_speed = *target_speed;
    }
}


// Blade direction is MOTOR_BLADE_FORWARD: low = normal (the only direction used
// until 2026-09), high = reverse. It only changes while the blade stands still:
// a new sign holds the PWM at 0 until the blade pulses (counted over 0.5 s in
// sensor.c) have been 0 for BLADE_STOP_TICKS ramp periods. Before this, a
// negative PWM went through uint16_t and setpwm() clamped it to 2047: full speed
// forward.
#define BLADE_STOP_TICKS 10 // x 50 ms
static uint8_t bladeStoppedTicks = 0;

static void bladeDirection(void) {
    bool reverse = GPIO_CHK_PIN(MOTOR_BLADE_FORWARD);
    bool wantReverse = setPWMBlade < 0;

    if (curPWMBlade == 0 && sensorMsg.motorPulseBlade == 0) {
        if (bladeStoppedTicks < BLADE_STOP_TICKS) bladeStoppedTicks++;
    } else {
        bladeStoppedTicks = 0;
    }
    if (setPWMBlade == 0 || wantReverse == reverse) return;

    if (curPWMBlade == 0 && bladeStoppedTicks >= BLADE_STOP_TICKS) {
        if (wantReverse) GPIO_SET_PIN(MOTOR_BLADE_FORWARD); else GPIO_CLR_PIN(MOTOR_BLADE_FORWARD);
    }
}

void motorcontrol_timer(TimerHandle_t xTimer) {
    uint16_t PWMBlade, PWMLeft, PWMRight = 0;

    // Quick ramp to setpwm
    ramp(&setPWMLeft, &curPWMLeft);
    ramp(&setPWMRight, &curPWMRight);

    // Blade: ramp the magnitude in the current direction; a sign change ramps
    // down to 0 and waits in bladeDirection() for the blade to stop.
    bladeDirection();
    bool bladeReverse = GPIO_CHK_PIN(MOTOR_BLADE_FORWARD);
    int16_t bladeTarget = ((setPWMBlade < 0) == bladeReverse) ? abs(setPWMBlade) : 0;
    ramp(&bladeTarget, &curPWMBlade);

    PWMBlade = curPWMBlade;
    // Make PWM value positive and correct the wheel direction
    if (curPWMLeft < 0) {
        PWMLeft = -curPWMLeft;
        GPIO_CLR_PIN(MOTOR_LEFT_FORWARD);
    } else {
        PWMLeft = curPWMLeft;
        GPIO_SET_PIN(MOTOR_LEFT_FORWARD);
    }

    if (curPWMRight < 0) {
        PWMRight = -curPWMRight;
        GPIO_SET_PIN(MOTOR_RIGHT_FORWARD);
    } else {
        PWMRight = curPWMRight;
        GPIO_CLR_PIN(MOTOR_RIGHT_FORWARD);
    }
    
    // if pwm = 0 brake on (wheels only, blade can freespin to stop since its going much faster)
    if (curPWMLeft == 0) {
        GPIO_CLR_PIN(MOTOR_LEFT_BRAKE);
    } else {
        GPIO_SET_PIN(MOTOR_LEFT_BRAKE);
    }

    if (curPWMRight == 0) {
        GPIO_CLR_PIN(MOTOR_RIGHT_BRAKE);
    } else {
        GPIO_SET_PIN(MOTOR_RIGHT_BRAKE);
    }
    setpwm(PWMBlade, PWMLeft, PWMRight);
}

/*
motorcontrol_timer
* fast ramp curpwm -> setpwm
* if setpwm 0, curpwm 0 but ticks increasing => brake on (?time? / until stopped?)

future:
We have to measure pulse/time (10ms? 100ms?) for some setpwm so we know speed curve on flat grass.
* check actual speed > set speed => lower setpwm temporarily
* check actual speed >much> set speed => software pwm brake on
* handle softwarepwm brake? (freertos timer or hardware timer?? we have TIMER2_IRQHandler in sensors - move this timer here?)

Also check current to motors and blade pulses/time if it is bogged down or not

 Initial test seems that reversing direction bit will brake less than full brake.
*/

// Reported as motorState whenever it changes: the real state, not the last
// request (which was MOTORREQ_SETSPEED almost always, even with motors off).
static xMotorRequestType motorStateNow(bool idle) {
    if (sensorMsg.emergancyStop) return MOTORREQ_EMGSTOP;
    if (sensorMsg.motorsEnabled) return MOTORREQ_ENABLE;
    return idle ? MOTORREQ_IDLE : MOTORREQ_DISABLE;
}

// Emergency causes that are still present (a reset is refused while any is).
static uint8_t emergencyCauses(void) {
    uint8_t causes = 0;
    if (GPIO_CHK_PIN(SENSOR_STOP)) causes |= EMG_STOP;
    if (!GPIO_CHK_PIN(SENSOR_LIFT)) causes |= EMG_LIFT;
    return causes;
}

void motorCtrl_Task(void *pvParameters) {

    xMotorRequestType lastState = -1;
    bool idle = true;
    bool spiLost = false;
    xMotorMsgType MotorMsg;

    MotorCtrl_Init();
    allStop();

    MotorMsg.action = MOTORREQ_IDLE;
    MotorMsg.pwm.left = 0;
    MotorMsg.pwm.right = 0;
    MotorMsg.pwm.blade = 0;
    xQueueSend(xMotorMsgQueue, &MotorMsg, xDelay25);

    // We need to wait for sensor to start.
    vTaskDelay(xDelay1000);
  

    TimerHandle_t timer2 = xTimerCreate("PWM Handler", pdMS_TO_TICKS(50), pdTRUE, NULL, motorcontrol_timer);
    xTimerStart(timer2, 0);

    for (;;) {
        

        if (sensorMsg.watchdogSPI > 5000) {
            allStop();
            if (!spiLost) debug("No command from ROS for 5 s: wheels and blade stopped");
            spiLost = true;
        } else {
            spiLost = false;
        }
        
        if (xQueueReceive(xMotorMsgQueue, &MotorMsg, xDelay25) == pdTRUE) {
            switch (MotorMsg.action) {
                case MOTORREQ_IDLE:
                    wdt_disable();
                    GPIO_CLR_PIN(MOTOR_MOSFET);
                    sensorMsg.motorsEnabled = false;
                    idle = true;
                    break;
                case MOTORREQ_ENABLE:
                    if (sensorMsg.emergancyStop) {
                        debug("MOTORREQ_ENABLE refused: emergency latched (EmgReason %u), send MOTORREQ_RESETEMG", sensorMsg.emergencyReason);
                        break;
                    }
                    if (sensorMsg.motorsEnabled) break;
                    allStop(); // no stale speed from before the enable
                    wdt_init();
                    
                // this need investigation... I tried a couple of different order of sequence:
                // enable controller, brake on, brake off (signals inverted I know)
                // This seem to work for all 3 controllers.
                    GPIO_SET_PIN(MOTOR_MOSFET);
                    vTaskDelay(xDelay200);
                    GPIO_CLR_PIN(MOTOR_BLADE_ENABLE);
                    vTaskDelay(xDelay50);
                    GPIO_CLR_PIN(MOTOR_BLADE_BRAKE);
                    vTaskDelay(xDelay50);
                    GPIO_SET_PIN(MOTOR_BLADE_BRAKE);

                    GPIO_CLR_PIN(MOTOR_LEFT_ENABLE);
                    GPIO_CLR_PIN(MOTOR_RIGHT_ENABLE);
                    vTaskDelay(xDelay50);
                    GPIO_CLR_PIN(MOTOR_LEFT_BRAKE);
                    GPIO_CLR_PIN(MOTOR_RIGHT_BRAKE);
                    vTaskDelay(xDelay50);
                    GPIO_SET_PIN(MOTOR_LEFT_BRAKE);
                    GPIO_SET_PIN(MOTOR_RIGHT_BRAKE);
                    sensorMsg.motorsEnabled = true;
                    idle = false;
                    break;
                case MOTORREQ_DISABLE:
                    wdt_disable(); 
                    allStop();
                    GPIO_CLR_PIN(MOTOR_MOSFET);
                    GPIO_SET_PIN(MOTOR_BLADE_BRAKE);
                    GPIO_SET_PIN(MOTOR_LEFT_BRAKE);
                    GPIO_SET_PIN(MOTOR_RIGHT_BRAKE);
                    sensorMsg.motorsEnabled = false;
                    idle = false;
                    break;
                case MOTORREQ_SETSPEED:
                    if (!sensorMsg.motorsEnabled || sensorMsg.emergancyStop) {
                        // Would otherwise be kept and run as soon as the motors come on.
                        setPWMBlade = setPWMLeft = setPWMRight = 0;
                        break;
                    }
                    if (sensorMsg.blockForward) {
                        if (MotorMsg.pwm.left > 0) MotorMsg.pwm.left = 0;
                        if (MotorMsg.pwm.right > 0) MotorMsg.pwm.right = 0;
                        if (MotorMsg.pwm.left < 0 || MotorMsg.pwm.right < 0) sensorMsg.blockForward = 0;
                        if (MotorMsg.pwm.left == 0 && MotorMsg.pwm.right == 0) sensorMsg.blockForward = 0;
                    } 

                    //setpwm(MotorMsg.pwm.blade, MotorMsg.pwm.left, MotorMsg.pwm.right);
                    setPWMBlade = MotorMsg.pwm.blade;
                    setPWMLeft = MotorMsg.pwm.left;
                    setPWMRight = MotorMsg.pwm.right;
                    break;
                case MOTORREQ_BRAKEON:
                    GPIO_CLR_PIN(MOTOR_BLADE_BRAKE);
                    GPIO_CLR_PIN(MOTOR_LEFT_BRAKE);
                    GPIO_CLR_PIN(MOTOR_RIGHT_BRAKE);
                    break;
                case MOTORREQ_BRAKEOFF:
                    GPIO_SET_PIN(MOTOR_BLADE_BRAKE);
                    GPIO_SET_PIN(MOTOR_LEFT_BRAKE);
                    GPIO_SET_PIN(MOTOR_RIGHT_BRAKE);
                    break;
                case MOTORREQ_EMGSTOP:
                    sensorMsg.emergencyReason |= MotorMsg.reason;
                    if (sensorMsg.emergancyStop) break; // already stopped, just add the reason
                    sensorMsg.emergancyStop = 1;
                    debug("EMERGENCY STOP, EmgReason %u", sensorMsg.emergencyReason);
                    wdt_disable();
                    allStop();
                    GPIO_CLR_PIN(MOTOR_BLADE_BRAKE);
                    GPIO_CLR_PIN(MOTOR_LEFT_BRAKE);
                    GPIO_CLR_PIN(MOTOR_RIGHT_BRAKE);
                    vTaskDelay(xDelay500);
                    GPIO_CLR_PIN(MOTOR_MOSFET);
                    sensorMsg.motorsEnabled = false;
                    break;
                case MOTORREQ_RESETEMG: {
                    uint8_t causes = emergencyCauses();
                    if (causes) {
                        debug("MOTORREQ_RESETEMG refused: cause still present (EmgReason %u)", causes);
                        sensorMsg.emergencyReason |= causes;
                        break;
                    }
                    if (!sensorMsg.emergancyStop) break;
                    sensorMsg.emergancyStop = 0;
                    sensorMsg.emergencyReason = 0;
                    allStop();
                    GPIO_CLR_PIN(MOTOR_MOSFET);
                    GPIO_SET_PIN(MOTOR_BLADE_BRAKE);
                    GPIO_SET_PIN(MOTOR_LEFT_BRAKE);
                    GPIO_SET_PIN(MOTOR_RIGHT_BRAKE);
                    sensorMsg.motorsEnabled = false;
                    idle = false;
                    break;
                }
                default:
                    break;
            }
        }

        xMotorRequestType state = motorStateNow(idle);
        if (state != lastState) {
            lastState = state;
            xJSONMessageType JSONMsg = {"motorState", {0}};
            strcpy(JSONMsg.value, MotorRequestStrings[state]);
            xQueueSend(xJSONMessageQueue, &JSONMsg, xDelay25);
        }
    }
}


/*





case EMGSTOP:
    setpwm(0, 0, 0);
    xScreenMsgType screenMsg;
    screenMsg.time=50;
    sprintf(screenMsg.text, "EMERGENCY STOP!");      
    xQueueSend(xScreenMsgQueue, &screenMsg, (TickType_t)0);
    vTaskDelay(xDelay250);
    GPIO_CLR_PIN(MOTOR_MOSFET); //<- set state power down motor instead
    break;

*/