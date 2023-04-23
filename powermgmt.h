#ifndef POWERMGMT_H
#define POWERMGMT_H

#include "FreeRTOS.h"
#include "queue.h"
#include "common.h"
#include "global.h"
/*
#pragma anon_unions
typedef struct
{
	xMessageType xType;
	union
	{
		struct
		{
			int32_t lChargeCurrent; // mA
			int32_t lBatteryVoltage; // mV
			int32_t lBatteryTemperature; // 1/100�C
		} measurement;
		struct
		{
			TickType_t xDelay;
		} shutdown;
	};
} xPowerMgmtMsg;

extern xQueueHandle xPowerMgmtMsgQueue;
*/


void powerMgmt_Task(void *pvParameters);

#endif // POWERMGMT_H
