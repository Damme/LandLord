
#include "global.h"
#include "queue.h"
#include "semphr.h"
#include "keypad.h"
#include "lcd.h"
#include "screen.h"

#include <stdio.h>

#ifndef MAIN_H
#define MAIN_H

#ifdef debugSemohosting
    // You need to change makefile
    // Need Trace.c, trace_impl.c in common too!
    extern void initialise_monitor_handles(void);
#endif

void ADC_IRQHandler(void);
void EINT3_IRQHandler(void);


static void task_CounterTest(void *pvParameters);
static void task_LCDUpdate(void *pvParameters);
static void task_Keypad(void *pvParameters);
static void semADCTask(void *pvParameters);
xSemaphoreHandle binADCTask;

static xQueueHandle xQueue = NULL;

#endif
