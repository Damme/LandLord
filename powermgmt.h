#ifndef POWERMGMT_H
#define POWERMGMT_H

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


typedef enum {
    Startup,
    Idle,
    CheckChargerInit,
    CheckCharger,
    StartCharging,
    Charging,
    BatteryCooldown,
    ChargingFinished,
    ChargingFailed,
    Shutdown,
    BatteryCold
} powerStates;

extern const char *powerStateStrings[];

void powerMgmt_Task(void *pvParameters);

#endif // POWERMGMT_H
