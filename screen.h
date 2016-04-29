#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "keypad.h"
#include "lcd.h"
#include "sensor.h"

#ifndef SCREEN_H
#define SCREEN_H


char buffer[50];

extern uint32_t testCounter;
//extern uint8_t pressed_key;
extern uint8_t debug1;
extern uint8_t debug2;
extern uint8_t debug3;
extern uint8_t debug4;

extern uint32_t ADC0;
extern uint32_t ADC1;
extern uint32_t ADC2;
extern uint32_t ADC3;
extern uint32_t ADC4;
extern uint32_t ADC5;
extern uint32_t ADC6;
extern uint32_t ADC7;



typedef struct menuItem_s {
    //uint8_t screenID;
    //enum enumScreenType screenType;
    char *itemName;
    void (*command)();
    const void *parm;
} menuItem_t;

typedef struct listItem_s {
    char *itemName;
    void (*command)();
    const void *parm;
} listItem_t;

typedef struct currentDisp_s {
    const menuItem_t *selected;
    void (*command)();
    const void *parm;
    uint8_t value;
} currentDisp_t;

uint8_t lcdCounter;
const listItem_t mainMenuList[7];
const listItem_t spindleMenuList[7];

const menuItem_t M0;
const menuItem_t M1;
const menuItem_t M2;

currentDisp_t currentDisplay;

void LCDInit(void);
void lcdUpdate(void);
void testfunc(void);
void motortest(void);

void menuBootfnc(void);
void menuBootWarnfnc(void);
void menuListfnc(void *ptr);

#endif