#ifndef MOTORCTRL_H
#define MOTORCTRL_H

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
			int32_t lMotorLeftCurrent; /* mA */
			int32_t lMotorRightCurrent; /* mA */
			int32_t lSpindleCurrent; /* mA */
		} measurement;
		struct
		{
			int32_t lMotorLeftSpeed;
			int32_t lMotorRightSpeed;
			int32_t lMotorLeftDistance;
			int32_t lMotorRightDistance;		
		} drive;
	};
} xMotorCtrlMsg;

extern xQueueHandle xMotorCtrlMsgQueue;

void task_MotorCtrl(void *pvParameters);

#endif // MOTORCTRL_H
