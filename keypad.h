#include "define.h"
#include "LPC17xx.h"
#include "define.h"
#include "freertos.h"
#include "task.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef KEYPAD_H
#define KEYPAD_H

extern TickType_t xDelay10;
extern TickType_t xDelay25;
extern TickType_t xDelay50;

const uint8_t tblKey[4][4];

uint8_t keyState, keyLastState;
uint8_t keyPessTime;


uint8_t keypadRow, keypadCol, keypadPressedKey;

void KeypadSetRow(bool row);
uint8_t keypadProcessTask();
uint8_t keypadGetState();
uint8_t keypadGetKey();
uint8_t keypadGetTime();

#endif