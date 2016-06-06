#include "define.h"
#include "global.h"

#ifndef SENSOR_H
#define SENSOR_H

bool sensorRain();
bool sensorLift();
bool sensorFront();
bool sensorCover();
bool sensorCharger();
uint8_t sensorDIP();
uint8_t sensorWireR();
uint8_t sensorWireL();

#endif
