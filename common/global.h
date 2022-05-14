#ifndef GLOBAL_H
#define GLOBAL_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

//int _write(int fd, char * str, int len);


//extern int stack_Keypad, stack_Counter, stack_LCD, stack_ADC;
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

extern xQueueHandle xSensorQueue;
extern xQueueHandle xScreenMsgQueue;
extern xQueueHandle xMotorMsgQueue;

typedef enum {
    MEASUREMENT_BATTERY = 0,
    MEASUREMENT_MOTORCURRENT,
    COMMAND_SHUTDOWN,
    COMMAND_STOP,
} xMessageType;

typedef struct debug_s {
    uint32_t portStat;
    uint32_t lastUpdate;
} debug_t;

typedef struct {
    uint32_t time;
    char text[32];
} xScreenMsgType;

typedef struct {
    bool stuck;
    bool stuck2;
    bool door;
    bool lift;
    bool collision;
    bool stop;
    bool cover; // ???
    bool rain;
    bool incharger;
    int32_t rainAnalog;
    int32_t batteryTemp;
    int32_t batteryVolt;
    int32_t batteryChargeCurrent;
    int32_t motorRCurrent;
    int32_t motorLCurrent;
    int32_t motorSCurrent;
    int32_t boardTemp;
    int16_t AccelX;
    int16_t AccelY;
    int16_t AccelZ;
    int16_t MotionYaw;
    int16_t MotionPitch;
    int16_t MotionRoll;
} xSensorMsgType;

typedef enum {
    BRAKE = 0,
    EMGSTOP,
    SETSPEED
} xMotorMessageType;

typedef struct {
    xMotorMessageType action;
    int16_t blade;
    int16_t left;
    int16_t right;
} xMotorMsgType;



extern volatile debug_t debugArray[5];
extern volatile uint8_t debugState;
extern volatile bool debugStateReset;
extern volatile uint32_t debug1;
extern volatile uint8_t debug2;
extern volatile uint8_t debug3;
extern volatile uint8_t debug4;

extern volatile uint32_t cpuID;


//int _write(int fd, char *ptr, int len);
void delay_uS(uint32_t uS);
void vAssertCalled( void );

//void IAP_get_device_UID(uint32_t UID_array[5]);

#endif

