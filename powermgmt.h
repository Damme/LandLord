#ifndef POWERMGMT_H
#define POWERMGMT_H

#include "FreeRTOS.h"
#include "queue.h"
#include "define.h"

#pragma anon_unions
typedef struct
{
	xMessageType xType;
	union
	{
		struct
		{
			int32_t lChargeCurrent; /* mA */
			int32_t lBatteryVoltage; /* mV */
			int32_t lBatteryTemperature; /* 1/100°C */
		} measurement;
		struct
		{
			TickType_t xDelay;
		} shutdown;
	};
} xPowerMgmtMsg;

extern xQueueHandle xPowerMgmtMsgQueue;

void task_PowerMgmt(void *pvParameters);

#endif // POWERMGMT_H
