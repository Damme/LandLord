#ifndef CONSOLE_H
#define CONSOLE_H

#include "FreeRTOS.h"
#include "queue.h"

#define sensorQUEUE_LENGTH          ( 10 )

void task_Console(void *pvParameters);

#endif
