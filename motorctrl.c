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


void motorCtrl_Task(void *pvParameters) {
    xMotorMsgType MotorMsg;
    xSensorMsgType SensorMsg;
    
    MotorCtrl_Init();
    setpwm(0, 0, 0);

    GPIO_SET_PIN(MOTOR_MOSFET);
   
    GPIO_CLR_PIN(MOTOR_BLADE_ENABLE);
    GPIO_CLR_PIN(MOTOR_LEFT_ENABLE);
    GPIO_CLR_PIN(MOTOR_RIGHT_ENABLE);
    vTaskDelay(xDelay100);
// seem to have to cycle brakes once to release them. 
    GPIO_CLR_PIN(MOTOR_BLADE_BRAKE);
    GPIO_CLR_PIN(MOTOR_LEFT_BRAKE);
    GPIO_CLR_PIN(MOTOR_RIGHT_BRAKE);
    vTaskDelay(xDelay250); 
    GPIO_SET_PIN(MOTOR_LEFT_BRAKE);
    GPIO_SET_PIN(MOTOR_RIGHT_BRAKE);
    GPIO_SET_PIN(MOTOR_BLADE_BRAKE);

    uint16_t setBladePWM = 0;
    uint16_t setLeftPWM = 0;
    uint16_t setRightPWM = 0;

    uint16_t curBladePWM = 0;
    uint16_t curLeftPWM = 0;
    uint16_t curRightPWM = 0;
    
    for (;;) {
        xQueuePeek(xSensorQueue, &SensorMsg, xDelay10);
        

        if (watchdogSPI > 5000) {
            setpwm(0,0,0);
            xScreenMsgType screenMsg;
            screenMsg.time=50;
            sprintf(screenMsg.text, "SPI comm lost!");      
            xQueueSend(xScreenMsgQueue, &screenMsg, (TickType_t)0);
            vTaskDelay(TicksPerMS * 2500);
        }/* else if (xQueueReceive(xMotorMsgQueue, &MotorMsg, xDelay25) == pdTRUE) {

            if (!SensorMsg.collision || !SensorMsg.lift) {
                if (MotorMsg.left > 0) MotorMsg.left = 0;
                if (MotorMsg.right > 0) MotorMsg.right = 0;
            }

            switch (MotorMsg.action) {
                case ENABLE:
                    GPIO_SET_PIN(MOTOR_MOSFET);
                    GPIO_CLR_PIN(MOTOR_BLADE_ENABLE);
                    GPIO_CLR_PIN(MOTOR_LEFT_ENABLE);
                    GPIO_CLR_PIN(MOTOR_RIGHT_ENABLE);
                    break;

                case SETSPEED:
                    if (MotorMsg.left < 0) {
                        GPIO_SET_PIN(MOTOR_LEFT_FORWARD);
                    } else {
                        MotorMsg.left = -MotorMsg.left;
                        GPIO_CLR_PIN(MOTOR_LEFT_FORWARD);                       
                    }

                    if (MotorMsg.right < 0) {
                        MotorMsg.right = -MotorMsg.right;
                        GPIO_CLR_PIN(MOTOR_RIGHT_FORWARD);
                    } else {
                        GPIO_SET_PIN(MOTOR_RIGHT_FORWARD);
                    }
                    
                    setpwm(MotorMsg.blade, MotorMsg.left, MotorMsg.right);
                    break;

                case EMGSTOP:
                    setpwm(0, 0, 0);
                    xScreenMsgType screenMsg;
                    screenMsg.time=50;
                    sprintf(screenMsg.text, "EMERGENCY STOP!");      
                    xQueueSend(xScreenMsgQueue, &screenMsg, (TickType_t)0);
                    vTaskDelay(xDelay250);
                    GPIO_CLR_PIN(MOTOR_MOSFET); //<- set state power down motor instead
                    break;

                case BRAKE:
                    setpwm(0, 0, 0);
                    break;
                
                default:
                    break;
            }
        }*/
        
        if (xQueueReceive(xMotorMsgQueue, &MotorMsg, xDelay25) == pdTRUE) {
            if (MotorMsg.action == BRAKE || MotorMsg.action == EMGSTOP) {
                setpwm(0, 0, 0);
                vTaskDelay(xDelay250);
                //GPIO_CLR_PIN(MOTOR_BLADE_BRAKE);
                //GPIO_CLR_PIN(MOTOR_LEFT_BRAKE);
                //GPIO_CLT_PIN(MOTOR_RIGHT_BRAKE);
                if (MotorMsg.action == EMGSTOP) {
                    vTaskDelay(xDelay250);
                    GPIO_CLR_PIN(MOTOR_MOSFET);
                    xScreenMsgType screenMsg;
                    screenMsg.time=50;
                    sprintf(screenMsg.text, "EMERGENCY STOP!");      
                    xQueueSend(xScreenMsgQueue, &screenMsg, (TickType_t)0);
                }
            }
            if (MotorMsg.action == SETSPEED) {
                vTaskDelay(xDelay25);
                // implement smoothing!

                if (MotorMsg.left < 0) {
                    MotorMsg.left = -MotorMsg.left;
                    GPIO_CLR_PIN(MOTOR_LEFT_FORWARD);
                } else {
                    GPIO_SET_PIN(MOTOR_LEFT_FORWARD);
                }
                if (MotorMsg.right < 0) {
                    MotorMsg.right = -MotorMsg.right;
                    GPIO_CLR_PIN(MOTOR_RIGHT_FORWARD);
                } else {
                    GPIO_SET_PIN(MOTOR_RIGHT_FORWARD);
                }
                
                setpwm(MotorMsg.blade, MotorMsg.left, MotorMsg.right);             
            }
        }
    }
}
