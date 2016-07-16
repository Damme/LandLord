#include <stdio.h>
#include "motorctrl.h"
#include "timers.h"

#define xDelay100  ((TickType_t)100 / portTICK_PERIOD_MS)

xQueueHandle xMotorCtrlMsgQueue;

int32_t lMotorLeftCurrent;
int32_t lMotorRightCurrent;
int32_t lSpindleCurrent;

void task_MotorCtrl(void *pvParameters)
{
    xMotorCtrlMsgQueue = xQueueCreate(20, sizeof(xMotorCtrlMsg));
    lMotorLeftCurrent = INT32_MIN;
    lMotorRightCurrent = INT32_MIN;
    lSpindleCurrent = INT32_MIN;

    LPC_PINCON->PINSEL4 |= (1 << 0);      //PWM1.1
    LPC_PINCON->PINSEL4 |= (1 << 2);      //PWM1.2
    LPC_PINCON->PINSEL4 |= (1 << 4);      //PWM1.3

    LPC_PWM1->PCR = 0;
    LPC_PWM1->PR = 12; // The TC is incremented every PR+1 cycles of PCLK.

    LPC_PWM1->MR0 = 1000; // 2khz
    LPC_PWM1->MR1 = 0; // PWM1
    LPC_PWM1->MR2 = 0; // PWM2
    LPC_PWM1->MR3 = 0; // PWM3
    LPC_PWM1->MCR = BIT(1);             // Reset on PWMMR0
    LPC_PWM1->LER = BIT(0) | BIT(1) | BIT(2) | BIT(3); // MR0 - MR3 enabled.

    LPC_PWM1->PCR = BIT(9) | BIT(10) | BIT(11); // PWMENA1 | PWMENA2 | PWMENA3
    LPC_PWM1->TCR = BIT(1);             // Counter Reset
    LPC_PWM1->TCR = BIT(0) | BIT(3);    //Counter Enable | PWM Enable

    // Spindle
    LPC_GPIO2->FIODIR |= PIN(13); /* P2.13 Enable=1 */
    LPC_GPIO3->FIODIR |= PIN(25); /* P3.25 Brake=1 */
    LPC_GPIO3->FIODIR |= PIN(26); /* P3.26 Forward=0/Reverse=1 */

    // Left Motor
    LPC_GPIO2->FIODIR |= (1 << 9); /* P2.9 Enable=1 */
    LPC_GPIO2->FIODIR |= (1 << 8); /* P2.8 Brake=1 */
    LPC_GPIO0->FIODIR |= (1 << 0); /* P0.0 Forward=0/Reverse=1 */

    // Right Motor
    LPC_GPIO2->FIODIR |= (1 << 4); /* P2.4 Enable=1 */
    LPC_GPIO2->FIODIR |= (1 << 5); /* P2.5 Brake=1 */
    LPC_GPIO2->FIODIR |= (1 << 6); /* P2.6 Forward=0/Reverse=1 */

    for (;;) {
        xMotorCtrlMsg msg;
        if (xQueueReceive(xMotorCtrlMsgQueue, &msg, xDelay100) == pdTRUE) {
            switch (msg.xType) {
                case (MEASUREMENT_MOTORCURRENT): {
                        if (msg.measurement.lMotorLeftCurrent != INT32_MIN) {
                            lMotorLeftCurrent = msg.measurement.lMotorLeftCurrent;
                            printf("Left Motor I: %4lumA\r\n", lMotorLeftCurrent);
                        }
                        if (msg.measurement.lMotorRightCurrent != INT32_MIN) {
                            lMotorRightCurrent = msg.measurement.lMotorRightCurrent;
                            printf("Right Motor I: %4lumA\r\n", lMotorRightCurrent);
                        }
                        if (msg.measurement.lSpindleCurrent != INT32_MIN) {
                            lSpindleCurrent = msg.measurement.lSpindleCurrent;
                            printf("Spindle I: %4lumA\r\n", lSpindleCurrent);
                        }
                        break;
                    }
                case (COMMAND_STOP): {
                        //	printf("Stop!\r\n");
                        // Disable all motors
                        LPC_GPIO2->FIOPIN &= ~PIN(4); //R
                        LPC_GPIO2->FIOPIN &= ~PIN(9);
                        LPC_GPIO2->FIOPIN &= ~PIN(13);

                        // Stop all PWM
                        LPC_PWM1->MR1 = 0; // PWM1
                        LPC_PWM1->MR2 = 0; // PWM2
                        LPC_PWM1->MR3 = 0; // PWM3
                        LPC_PWM1->LER = BIT(0) | BIT(1) | BIT(2) | BIT(3); // MR0 - MR3 enabled.

                        // Enable Brake on all motors
                        LPC_GPIO2->FIOPIN |= PIN(5); //R
                        LPC_GPIO2->FIOPIN |= PIN(8);
                        LPC_GPIO3->FIOPIN |= PIN(25);
                        break;
                    }
                default:
                    printf("Unknown msg\r\n");
            }
        }
    }
}
