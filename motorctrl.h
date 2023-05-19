#ifndef MOTORCTRL_H
#define MOTORCTRL_H

#include "FreeRTOS.h"
#include "queue.h"
#include "global.h"


void motorCtrl_Task(void *pvParameters);
void allStop();

#endif // MOTORCTRL_H
