#ifndef MOTORCTRL_H
#define MOTORCTRL_H

#include "FreeRTOS.h"

void motorCtrl_Task(void *pvParameters);
void allStop();
void motionSensor_TickFromISR(BaseType_t *woken); // every 1 ms from the TIMER2 ISR

#endif // MOTORCTRL_H
