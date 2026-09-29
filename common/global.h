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
#include "stream_buffer.h"

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
extern StreamBufferHandle_t SPI0TxStreamBuffer;
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
    uint8_t emergencyReason; // EMG_* bits, latched until MOTORREQ_RESETEMG
    bool motorsEnabled;      // real state: MOTOR_MOSFET on and controllers enabled
    int32_t globalticksms;
    uint32_t bumps;          // debounced bumper/collision presses since boot
    int32_t watchdogSPI;
    int32_t rainAnalog;
    int32_t batteryTemp;
    int32_t batteryVolt;
    int32_t batteryChargeCurrent;
    int32_t boardTemp;
    double accelX;
    double accelY;
    double accelZ;
    double gyroYaw;
    double gyroPitch;
    double gyroRoll;
    uint16_t currentPWMRight;
    uint16_t currentPWMLeft;
    uint16_t currentPWMBlade;
    uint16_t motorCurrentRight;
    uint16_t motorCurrentLeft;
    uint16_t motorCurrentBlade;
    uint32_t motorPulseLeft;
    uint32_t motorPulseRight;
    uint32_t motorPulseBlade;
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

// Emergency reasons (sensorMsg.emergencyReason, "EmgReason" in MotorPulse)
#define EMG_TILT  (1 << 0) // accelZ < 5000 with the blade on for 0.5 s
#define EMG_STOP  (1 << 1) // STOP key
#define EMG_LIFT  (1 << 2) // lift sensor with the motors on (debounced)

typedef struct {
    xMotorRequestType action;
    uint8_t reason; // EMG_* for MOTORREQ_EMGSTOP
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
void wdt_disable();
  
//int _write(int fd, char *ptr, int len);
void delay_uS(uint32_t uS);
void vAssertCalled( void );
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);


//void IAP_get_device_UID(uint32_t UID_array[5]);

#endif

