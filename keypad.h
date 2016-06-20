#include "global.h"

#ifndef KEYPAD_H
#define KEYPAD_H

void KeypadSetRow(bool row);
uint8_t keypadProcessTask(void);
uint8_t keypadGetState(void);
uint8_t keypadGetKey(void);
uint8_t keypadGetTime(void);

#endif
