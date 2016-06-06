#include "keypad.h"
#include "lcd.h"
#include "sensor.h"
#include "global.h"

#ifndef SCREEN_H
#define SCREEN_H


char buffer[50];

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
