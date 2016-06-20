#include "define.h"
#include "global.h"

#ifndef SENSOR_H
#define SENSOR_H

bool sensorRain(void);
bool sensorLift(void);
bool sensorFront(void);
bool sensorCover(void);
bool sensorCharger(void);
uint8_t sensorDIP(void);
uint8_t sensorWireR(void);
uint8_t sensorWireL(void);

#endif
