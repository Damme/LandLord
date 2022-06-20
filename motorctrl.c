#include "motorctrl.h"
#include "timers.h"
#include <stdio.h>

/*
PWM1 BLADE
PWM4 LEFT
PWM5 RIGHT
*/

void setpwm(uint16_t blade, uint16_t left, uint16_t right) {
    if (blade > 1000) blade=1000;
    if (left > 1000) left=1000;
    if (right > 1000) right=1000;

    LPC_PWM1->MR1 = blade;
    LPC_PWM1->MR4 = left;
    LPC_PWM1->MR5 = right;
    LPC_PWM1->LER |= (1<<1) | (1<<4) | (1<<5);    
}


void motorCtrl_Task(void *pvParameters) {
    xMotorMsgType MotorMsg;
    
    MotorCtrl_Init();
    
    GPIO_SET_PIN(MOTOR_MOSFET);
    setpwm(0, 0, 0);
// don't think we need the delays    
    vTaskDelay(xDelay50);
    GPIO_CLR_PIN(MOTOR_BLADE_ENABLE);
    GPIO_CLR_PIN(MOTOR_LEFT_ENABLE);
    GPIO_CLR_PIN(MOTOR_RIGHT_ENABLE);
    vTaskDelay(xDelay50);
// seem to have to cycle brakes once to release them. 
    GPIO_CLR_PIN(MOTOR_BLADE_BRAKE);
    GPIO_CLR_PIN(MOTOR_LEFT_BRAKE);
    GPIO_CLR_PIN(MOTOR_RIGHT_BRAKE);
// might  need this delay
    vTaskDelay(xDelay50);
    GPIO_SET_PIN(MOTOR_BLADE_BRAKE);
    GPIO_SET_PIN(MOTOR_LEFT_BRAKE);
    GPIO_SET_PIN(MOTOR_RIGHT_BRAKE);
    
    for (;;) {
        if (xQueueReceive(xMotorMsgQueue, &MotorMsg, xDelay25) == pdTRUE) {
            if (MotorMsg.action == BRAKE || MotorMsg.action == EMGSTOP) {
                setpwm(0, 0, 0);
                vTaskDelay(xDelay250);
                GPIO_SET_PIN(MOTOR_BLADE_BRAKE);
                GPIO_SET_PIN(MOTOR_LEFT_BRAKE);
                GPIO_SET_PIN(MOTOR_RIGHT_BRAKE);
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
                /*xScreenMsgType screenMsg;
                screenMsg.time=20;
                sprintf(screenMsg.text, "SETPWM %i %i %i", MotorMsg.blade, MotorMsg.left, MotorMsg.right);
                xQueueSend(xScreenMsgQueue, &screenMsg, (TickType_t)0);*/
                printf("SETPWM %i %i %i\n", MotorMsg.blade, MotorMsg.left, MotorMsg.right);
                
                //setpwm(MotorMsg.blade, MotorMsg.left, MotorMsg.right);
                
                // !!!! check direction change !!!! NEGATIVE NUMBERS !!!!
                
            }
        }
    }
}
