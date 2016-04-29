
#include "define.h"
#include "LPC17xx.h"
#include "FreeRTOS.h"
#include "task.h"
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

static xQueueHandle xQueue = NULL;

xSemaphoreHandle binADCTask;


TickType_t xDelay1 = TicksPerMS * 1;
TickType_t xDelay10 = TicksPerMS * 10;
TickType_t xDelay25 = TicksPerMS * 25;
TickType_t xDelay50 = TicksPerMS * 50;
TickType_t xDelay100 = TicksPerMS * 100;
TickType_t xDelay200 = TicksPerMS * 200;
TickType_t xDelay150 = TicksPerMS * 150;
TickType_t xDelay250 = TicksPerMS * 250;
TickType_t xDelay500 = TicksPerMS * 500;
TickType_t xDelay1000 = TicksPerMS * 1000;


uint8_t debug1 = 0;
uint8_t debug2 = 0;
uint8_t debug3 = 0;
uint8_t debug4 = 0;

uint32_t ADC0 = 0;
uint32_t ADC1 = 0;
uint32_t ADC2 = 0;
uint32_t ADC3 = 0;
uint32_t ADC4 = 0;
uint32_t ADC5 = 0;
uint32_t ADC6 = 0;
uint32_t ADC7 = 0;


#endif
