#include "keypad.h"
#include "lcd.h"
#include "sensor.h"

#ifndef SCREEN_H
#define SCREEN_H

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

void LCDInit(void);
void lcdUpdate(void);
void testfunc(void);
void motortest(void);

void menuBootfnc(void);
void menuBootWarnfnc(void);
void menuListfnc(void *ptr);

#endif
