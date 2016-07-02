#ifndef KEYPAD_H
#define KEYPAD_H

#include <stdint.h>
#include <stdbool.h>

void KeypadSetRow(bool row);
uint8_t keypadProcessTask(void);
uint8_t keypadGetState(void);
uint8_t keypadGetKey(void);
uint8_t keypadGetTime(void);

static void task_Keypad(void *pvParameters);

#endif
