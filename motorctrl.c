#include "motorctrl.h"
#include "timers.h"
#include <stdio.h>

int16_t setPWMBlade, setPWMLeft, setPWMRight = 0;
int16_t curPWMBlade, curPWMLeft, curPWMRight = 0;
int32_t tiltCounter = 0;

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


void motionSensor_Timer(TimerHandle_t xTimer) {
    if (!GPIO_CHK_PIN(SENSOR_STUCK) || !GPIO_CHK_PIN(SENSOR_STUCK2) || !GPIO_CHK_PIN(SENSOR_LIFT) || !GPIO_CHK_PIN(SENSOR_COLLISION))
        sensorMsg.blockForward = 1;

    // If robot tilted too much / upside down and blade on emergancy stop!
    if (sensorMsg.accelZ < 5000 && sensorMsg.currentPWMBlade > 0) {
        tiltCounter++;
        if (tiltCounter > 500) {
            debug(" TILT EMG STOP");
        xMotorMsgType MotorMsg;
        MotorMsg.action = MOTORREQ_EMGSTOP;
        MotorMsg.pwm.left = 0;
        MotorMsg.pwm.right = 0;
        MotorMsg.pwm.blade = 0;
        xQueueSend(xMotorMsgQueue, &MotorMsg, xDelay500);
    }
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


void motorcontrol_timer(TimerHandle_t xTimer) {
    uint16_t PWMBlade, PWMLeft, PWMRight = 0;

    // Quick ramp to setpwm
    ramp(&setPWMLeft, &curPWMLeft);
    ramp(&setPWMRight, &curPWMRight);
    ramp(&setPWMBlade, &curPWMBlade);
    
    /*curPWMLeft = setPWMLeft;
    curPWMRight = setPWMRight;
    */
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

void motorCtrl_Task(void *pvParameters) {

    xMotorRequestType lastState = -1;
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
  
    TimerHandle_t timer1 = xTimerCreate("Motion sensors", TicksPerMS, pdTRUE, NULL, motionSensor_Timer);
    xTimerStart(timer1, 0);

    TimerHandle_t timer2 = xTimerCreate("PWM Handler", pdMS_TO_TICKS(50), pdTRUE, NULL, motorcontrol_timer);
    xTimerStart(timer2, 0);

    for (;;) {
        

        if (sensorMsg.watchdogSPI > 5000) {
            allStop();
            //xScreenMsgType screenMsg;
            //screenMsg.time=50;
            //sprintf(screenMsg.text, "ROS comms lost!");
            //xQueueSend(xScreenMsgQueue, &screenMsg, (TickType_t)0);
            // Queue emgstop?
        }
        
        if (xQueueReceive(xMotorMsgQueue, &MotorMsg, xDelay25) == pdTRUE) {
            if (MotorMsg.action != lastState) {
                lastState = MotorMsg.action;
                xJSONMessageType JSONMsg = {"motorState", {0}};
                strcpy(JSONMsg.value, MotorRequestStrings[MotorMsg.action]);
                xQueueSend(xJSONMessageQueue, &JSONMsg, xDelay25);
            }

            switch (MotorMsg.action) {
                case MOTORREQ_IDLE:
                    GPIO_CLR_PIN(MOTOR_MOSFET);
                    break;
                case MOTORREQ_ENABLE:
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
                    break;
                case MOTORREQ_DISABLE:
                    allStop();
                    GPIO_CLR_PIN(MOTOR_MOSFET);
                    GPIO_SET_PIN(MOTOR_BLADE_BRAKE);
                    GPIO_SET_PIN(MOTOR_LEFT_BRAKE);
                    GPIO_SET_PIN(MOTOR_RIGHT_BRAKE);
                    break;
                case MOTORREQ_SETSPEED:
                    if (sensorMsg.blockForward) {
                        if (MotorMsg.pwm.left > 0) MotorMsg.pwm.left = 0;
                        if (MotorMsg.pwm.right > 0) MotorMsg.pwm.right = 0;
                        if (MotorMsg.pwm.left < 0 || MotorMsg.pwm.right < 0) sensorMsg.blockForward = 0;
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
                    allStop();
                    GPIO_CLR_PIN(MOTOR_BLADE_BRAKE);
                    GPIO_CLR_PIN(MOTOR_LEFT_BRAKE);
                    GPIO_CLR_PIN(MOTOR_RIGHT_BRAKE);
                    sensorMsg.emergancyStop = 1;
                    vTaskDelay(xDelay500);
                    GPIO_CLR_PIN(MOTOR_MOSFET);
                    // Queue idle?
                    break;
                case MOTORREQ_RESETEMG:
                    sensorMsg.emergancyStop = 0;
                    GPIO_CLR_PIN(MOTOR_MOSFET);
                    GPIO_SET_PIN(MOTOR_BLADE_BRAKE);
                    GPIO_SET_PIN(MOTOR_LEFT_BRAKE);
                    GPIO_SET_PIN(MOTOR_RIGHT_BRAKE);
                    break;
            }
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