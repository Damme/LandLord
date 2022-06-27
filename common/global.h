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

extern xQueueHandle xBoundaryMsgQueue;

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
    bool door2;
    bool rain;
    bool incharger;
    bool batteryCellLow;
    bool batteryCellHigh;
    int32_t rainAnalog;
    int32_t batteryTemp;
    int32_t batteryVolt;
    int32_t batteryChargeCurrent;
    int32_t motorRCurrent;
    int32_t motorLCurrent;
    int32_t motorBRpm;
    int32_t boardTemp;
    int16_t AccelX;
    int16_t AccelY;
    int16_t AccelZ;
    int16_t MotionYaw;
    int16_t MotionPitch;
    int16_t MotionRoll;
    int16_t MagX;
    int16_t MagY;
    int16_t MagZ;
    uint16_t CurrentPWMRight;
    uint16_t CurrentPWMLeft;
    uint16_t CurrentPWMSpindle;
    int32_t motorpulseleft;
    int32_t motorpulseright;
    int32_t motorpulseblade;
} xSensorMsgType;

// V0.7#S-239568|+502936#N035|143#C073#
typedef struct {
    char fwver[4];
    char reserved0[2];

    char sleft[7];
    char reserved1;

    char sright[7];
    char reserved2[2];

    char nleft[3];
    char reserved3;

    char nright[3];
    char reserved4[2];

    char checksum[3];
    char reserved5[4];
    //char buf[];
} xBoundaryMsgType;

typedef enum {
    ENABLE = 0,
    SETSPEED,
    STOP,
    BRAKE,
    EMGSTOP,
    STARTBLADE,
    STOPBLADE        
} xMotorMessageType;

typedef struct {
    xMotorMessageType action;
    int16_t blade;
    int16_t left;
    int16_t right;
} xMotorMsgType;


extern volatile debug_t debugArray[5];

extern volatile uint32_t cpuID;

extern volatile TaskHandle_t xHandle[10];
extern volatile uint8_t taskcounter;
extern volatile uint64_t globaltickms;
extern volatile uint32_t watchdogSPI;

//int _write(int fd, char *ptr, int len);
void delay_uS(uint32_t uS);
void vAssertCalled( void );

//void IAP_get_device_UID(uint32_t UID_array[5]);

#endif

