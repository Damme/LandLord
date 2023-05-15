#include "motorctrl.h"
#include "timers.h"
#include <stdio.h>

void setpwm(uint16_t blade, uint16_t left, uint16_t right) {
    if (blade > 2047) blade=2047;
    if (left > 2047) left=2047;
    if (right > 2047) right=2047;

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


void motionSensor_Timer(void) {
    if (!GPIO_CHK_PIN(SENSOR_STUCK) || !GPIO_CHK_PIN(SENSOR_STUCK2) || !GPIO_CHK_PIN(SENSOR_LIFT) || !GPIO_CHK_PIN(SENSOR_COLLISION))
        sensorMsg.blockForward = 1;

    // If robot tilted too much / upside down and blade on emergancy stop!
    if (sensorMsg.accelZ < 500 && sensorMsg.currentPWMBlade > 0) {
        xMotorMsgType MotorMsg;
        MotorMsg.action = MOTORREQ_EMGSTOP;
        MotorMsg.pwm.left = 0;
        MotorMsg.pwm.right = 0;
        MotorMsg.pwm.blade = 0;
        xQueueSend(xMotorMsgQueue, &MotorMsg, xDelay500);
    }
}

void motorCtrl_Task(void *pvParameters) {

    xMotorRequestType lastState = -1;
    xMotorMsgType MotorMsg;

    MotorCtrl_Init();
    setpwm(0, 0, 0);

    // We need to wait for sensor to start.
    vTaskDelay(xDelay1000);
  
    TimerHandle_t timer1 = xTimerCreate("Motion sensors", TicksPerMS, pdTRUE, NULL, motionSensor_Timer);
    xTimerStart(timer1, 0);

    for (;;) {

        if (sensorMsg.watchdogSPI > 4000) {
            setpwm(0,0,0);
            xScreenMsgType screenMsg;
            screenMsg.time=50;
            sprintf(screenMsg.text, "ROS comms lost!");
            xQueueSend(xScreenMsgQueue, &screenMsg, (TickType_t)0);
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
                    GPIO_SET_PIN(MOTOR_MOSFET);
                    GPIO_CLR_PIN(MOTOR_BLADE_ENABLE);
                    GPIO_CLR_PIN(MOTOR_LEFT_ENABLE);
                    GPIO_CLR_PIN(MOTOR_RIGHT_ENABLE);
                    vTaskDelay(xDelay100); 
                    // this needs investigation about the sequence to intiate the motor controllers. - mc33035dw
                    // pin should be inverted in HAL - clr = brake on, set =  brake off
                    GPIO_CLR_PIN(MOTOR_BLADE_BRAKE);
                    GPIO_CLR_PIN(MOTOR_LEFT_BRAKE);
                    GPIO_CLR_PIN(MOTOR_RIGHT_BRAKE);
                    vTaskDelay(xDelay250);
                    GPIO_SET_PIN(MOTOR_BLADE_BRAKE);
                    GPIO_SET_PIN(MOTOR_LEFT_BRAKE);
                    GPIO_SET_PIN(MOTOR_RIGHT_BRAKE);
                    break;
                case MOTORREQ_DISABLE:
                    setpwm(0,0,0);
                    GPIO_CLR_PIN(MOTOR_MOSFET);
                    GPIO_SET_PIN(MOTOR_BLADE_BRAKE);
                    GPIO_SET_PIN(MOTOR_LEFT_BRAKE);
                    GPIO_SET_PIN(MOTOR_RIGHT_BRAKE);
                    break;
                case MOTORREQ_SETSPEED:
                // Check for overspeed ticks/100ms - is it possible to brake short pulses or even reversing direction with pwm 0 for soft brake?
                // Initial test seems that reversing direction bit will brake less than full brake.

                    if (sensorMsg.blockForward) {
                        if (MotorMsg.pwm.left > 0) MotorMsg.pwm.left = 0;
                        if (MotorMsg.pwm.right > 0) MotorMsg.pwm.right = 0;
                        if (MotorMsg.pwm.left < 0 || MotorMsg.pwm.right < 0) sensorMsg.blockForward = 0;
                    } 

                    if (MotorMsg.pwm.left < 0) {
                        MotorMsg.pwm.left = -MotorMsg.pwm.left;
                        GPIO_CLR_PIN(MOTOR_LEFT_FORWARD);
                    } else {
                        GPIO_SET_PIN(MOTOR_LEFT_FORWARD);
                    }

                    if (MotorMsg.pwm.right < 0) {
                        MotorMsg.pwm.right = -MotorMsg.pwm.right;
                        GPIO_SET_PIN(MOTOR_RIGHT_FORWARD);
                    } else {
                        GPIO_CLR_PIN(MOTOR_RIGHT_FORWARD);
                    }

                    setpwm(MotorMsg.pwm.blade, MotorMsg.pwm.left, MotorMsg.pwm.right);
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
                    setpwm(0,0,0);
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