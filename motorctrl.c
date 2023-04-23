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

typedef enum {
    Idle,
    EnableMotors,
    DisableMotors,
    RemoteControl,
    Stop,
    EmergencyStop,
    Brake
} motorStates;

bool block_forward = 0;

void motionSensor_Timer(void) {
    if (!GPIO_CHK_PIN(SENSOR_STUCK) || !GPIO_CHK_PIN(SENSOR_STUCK2) || !GPIO_CHK_PIN(SENSOR_LIFT) || !GPIO_CHK_PIN(SENSOR_COLLISION))
        block_forward = 1;
}

void motorCtrl_Task(void *pvParameters) {
    motorStates motorState = Idle;

    xMotorMsgType MotorMsg;
    xSensorMsgType SensorMsg;

    BaseType_t hasMotorMsg = pdFAIL;
        
    MotorCtrl_Init();
    setpwm(0, 0, 0);


// TODO TODO fix logic for motor_mosfet - power consumtion during idle.
    GPIO_SET_PIN(MOTOR_MOSFET);
// TODO TODO fix logic for motor_mosfet - power consumtion during idle.


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
    // or put this in the timer that counts wheel pulses
    TimerHandle_t timer1 = xTimerCreate("Motion sensors", TicksPerMS, pdTRUE, NULL, motionSensor_Timer);
    xTimerStart(timer1, 0);
    
    for (;;) {
        
        hasMotorMsg = xQueueReceive(xMotorMsgQueue, &MotorMsg, pdMS_TO_TICKS(50));
        

        if ((hasMotorMsg == pdTRUE) && (MotorMsg.action == BUTTON)) {
            switch (MotorMsg.button.pressed) {
                case 1: // ☓
                    if (GPIO_CHK_PIN(MOTOR_MOSFET)) {
                        GPIO_CLR_PIN(MOTOR_MOSFET);
                    } else {
                        GPIO_SET_PIN(MOTOR_MOSFET);
                    }
                    break;
                case 2: // ◯
                    break;
                case 3: // △
                    if (motorState == RemoteControl) {
                        motorState = Idle;
                    } else if (motorState == Idle) {
                        motorState = RemoteControl;
                    }
                    break;
                case 4: // ▯□
                    break;
            }
        }

        switch(motorState) {
            case Idle:
                // if currspeed != 0 then
                setpwm(0,0,0);
                break;
            case RemoteControl:
                if (watchdogSPI > 4000) {
                    setpwm(0,0,0);
                    xScreenMsgType screenMsg;
                    screenMsg.time=50;
                    sprintf(screenMsg.text, "SPI comm lost!");      
                    xQueueSend(xScreenMsgQueue, &screenMsg, (TickType_t)0);
                    printf(screenMsg.text);
                    motorState = Idle;
                } else if ((hasMotorMsg == pdTRUE) && (MotorMsg.action == SETSPEED)) {
                    
                    if (block_forward) {
                        if (MotorMsg.pwm.left > 0) MotorMsg.pwm.left = 0;
                        if (MotorMsg.pwm.right > 0) MotorMsg.pwm.right = 0;
                        if (MotorMsg.pwm.left < 0 || MotorMsg.pwm.right < 0) block_forward = 0;
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
                }
                break;
        }


#if 0
        //xQueuePeek(xSensorQueue, &SensorMsg, xDelay10);
/*        GPIO_CHK_PIN()
#define SENSOR_STUCK       (GPIO_TYPE(PORT_2, PIN_22, FUNC_0))
#define SENSOR_STUCK2      (GPIO_TYPE(PORT_1, PIN_22, FUNC_0))
#define SENSOR_LIFT        (GPIO_TYPE(PORT_2, PIN_16, FUNC_0)) // or Collision
#define SENSOR_COLLISION   (GPIO_TYPE(PORT_4, PIN_1, FUNC_0)) // Or Lift!
#define SENSOR_STOP        (GPIO_TYPE(PORT_1, PIN_25, FUNC_0))
*/
        if (watchdogSPI > 5000) {
            setpwm(0,0,0);
            xScreenMsgType screenMsg;
            screenMsg.time=50;
            sprintf(screenMsg.text, "SPI comm lost!");      
            xQueueSend(xScreenMsgQueue, &screenMsg, (TickType_t)0);
            vTaskDelay(TicksPerMS * 2500);
        } else if (xQueueReceive(xMotorMsgQueue, &MotorMsg, xDelay25) == pdTRUE) {
            
            /*if (!SensorMsg.stuck || !SensorMsg.stuck2 || !SensorMsg.collision || !SensorMsg.lift) {
                if (MotorMsg.left > 0) MotorMsg.left = 0;
                if (MotorMsg.right > 0) MotorMsg.right = 0;
            }*/

            switch (MotorMsg.action) {
                case ENABLE:
                    GPIO_SET_PIN(MOTOR_MOSFET);
                    GPIO_CLR_PIN(MOTOR_BLADE_ENABLE);
                    GPIO_CLR_PIN(MOTOR_LEFT_ENABLE);
                    GPIO_CLR_PIN(MOTOR_RIGHT_ENABLE);
                    break;

                case SETSPEED:
                    vTaskDelay(xDelay25); // what's this for?
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
        }
        memset(&MotorMsg, 0, sizeof(xMotorMsgType));
        /*
        
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
        }*/
#endif
    }
}
