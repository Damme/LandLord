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
#include "message_buffer.h"

extern TickType_t xDelay1;
extern TickType_t xDelay5;
extern TickType_t xDelay10;
extern TickType_t xDelay25;
extern TickType_t xDelay50;
extern TickType_t xDelay100;
extern TickType_t xDelay200;
extern TickType_t xDelay150;
extern TickType_t xDelay250;
extern TickType_t xDelay500;
extern TickType_t xDelay1000;
extern TickType_t xDelay2000;

extern xQueueHandle xScreenMsgQueue;
extern xQueueHandle xMotorMsgQueue;
extern xQueueHandle xBoundaryMsgQueue;
extern xQueueHandle xJSONMessageQueue;
extern MessageBufferHandle_t SPI0RxMessageBuffer;
extern MessageBufferHandle_t SPI0TxMessageBuffer;
extern xQueueHandle RosTxQueue;

typedef struct {
    uint32_t time;
    char text[32];
} xScreenMsgType;

typedef struct {
    bool stuck;
    bool stuck2;
    bool door;
    bool door2;
    bool lift;
    bool collision;
    bool stop;
    bool rain;
    bool batteryCellLow;
    bool batteryCellHigh;
    bool inCharger;
    bool blockForward;
    bool emergancyStop;
    int32_t globalticksms;
    int32_t watchdogSPI;
    int32_t rainAnalog;
    int32_t batteryTemp;
    int32_t batteryVolt;
    int32_t batteryChargeCurrent;
    int32_t boardTemp;
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t gyroYaw;
    int16_t gyroPitch;
    int16_t gyroRoll;
    uint16_t currentPWMRight;
    uint16_t currentPWMLeft;
    uint16_t currentPWMBlade;
    uint16_t motorCurrentRight;
    uint16_t motorCurrentLeft;
    uint16_t motorCurrentBlade;
    int32_t motorPulseLeft;
    int32_t motorPulseRight;
    int32_t motorPulseBlade;
} SensorType;



// V0.7#S-239568|+502936#N035|143#C073#
typedef struct {
    char fwver[4];
    char reserved0[2];

    char sright[7];
    char reserved1;

    char sleft[7];
    char reserved2[2];

    char nright[3];
    char reserved3;

    char nleft[3];
    char reserved4[2];

    char checksum[3];
    char reserved5[4];
    //char buf[];
} xBoundaryMsgType;

typedef struct {
    char topic[20];
    char value[20];
} xJSONMessageType;

typedef enum {
    MOTORREQ_IDLE = 0,
    MOTORREQ_ENABLE,
    MOTORREQ_DISABLE,
    MOTORREQ_SETSPEED,
    MOTORREQ_BRAKEON,
    MOTORREQ_BRAKEOFF,
    MOTORREQ_EMGSTOP,
    MOTORREQ_RESETEMG
} xMotorRequestType;

typedef struct {
    xMotorRequestType action;
    struct {
        int16_t blade;
        int16_t left;
        int16_t right;
    } pwm;
} xMotorMsgType;

/*
#pragma anon_unions
typedef struct {
    xMotorMessageType action;
    union {
        struct {
            int16_t blade;
            int16_t left;
            int16_t right;
        } pwm;
        struct {
            uint32_t pressed;
        } button;
    };
} xMotorMsgType;
*/

extern volatile uint32_t cpuID;

extern volatile TaskHandle_t xHandle[15];
extern volatile uint8_t taskcounter;

extern SensorType sensorMsg;


void wdt_reset();
void wdt_init(); 
  
//int _write(int fd, char *ptr, int len);
void delay_uS(uint32_t uS);
void vAssertCalled( void );
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);


//void IAP_get_device_UID(uint32_t UID_array[5]);

#endif

