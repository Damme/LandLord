#ifndef GLOBAL_H
#define GLOBAL_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "define.h"
#include "LPC17xx.h"
#include "FreeRTOS.h"
#include "task.h"

extern int stack_Keypad, stack_Counter, stack_LCD, stack_ADC;
extern bool run;

extern TickType_t xDelay1;
extern TickType_t xDelay10;
extern TickType_t xDelay25;
extern TickType_t xDelay50;
extern TickType_t xDelay100;
extern TickType_t xDelay200;
extern TickType_t xDelay150;
extern TickType_t xDelay250;
extern TickType_t xDelay500;
extern TickType_t xDelay1000;

typedef struct debug_s {
    uint32_t portStat;
    uint32_t lastUpdate;
} debug_t;

extern volatile debug_t debugArray[5];
extern volatile uint8_t debugState;
extern volatile bool debugStateReset;
extern volatile uint32_t debug1;
extern volatile uint8_t debug2;
extern volatile uint8_t debug3;
extern volatile uint8_t debug4;
extern volatile uint32_t ADC0;
extern volatile uint32_t ADC1;
extern volatile uint32_t ADC2;
extern volatile uint32_t ADC3;
extern volatile uint32_t ADC4;
extern volatile uint32_t ADC5;
extern volatile uint32_t ADC6;
extern volatile uint32_t ADC7;
extern volatile uint32_t delayuS;
extern volatile uint32_t delaymS;


#endif
