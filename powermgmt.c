#include <stdio.h>
#include "powermgmt.h"
#include "timers.h"

#define xDelay100  ((TickType_t)100 / portTICK_PERIOD_MS)

xQueueHandle xPowerMgmtMsgQueue;

int32_t lChargeCurrent;
int32_t lBatteryVoltage;
int32_t lBatteryTemperature;

void task_PowerMgmt(void *pvParameters)
{
	xPowerMgmtMsgQueue = xQueueCreate(20, sizeof(xPowerMgmtMsg));
	lChargeCurrent = INT32_MIN;
	lBatteryVoltage = INT32_MIN;
	lBatteryTemperature = INT32_MIN;
	
	for (;;)
	{
		xPowerMgmtMsg msg;
		if (xQueueReceive(xPowerMgmtMsgQueue, &msg, xDelay100) == pdTRUE)
		{
			switch (msg.xType)
			{
				case (MEASUREMENT_BATTERY):
				{
					/* https://hackaday.io/project/6717-project-landlord/discussion-58892 */
					if (msg.measurement.lChargeCurrent != INT32_MIN)
					{
						lChargeCurrent = msg.measurement.lChargeCurrent;
						printf("Charge I: %4umA\r\n", lChargeCurrent);
					}
					if (msg.measurement.lBatteryVoltage != INT32_MIN)
					{
						// orig fw: v16_batteryLowV = 237; - 23.7V (fixed point) - 3.386V per cell
						// orig fw: v18_batteryFullCharge = 287; - 28.7V (fixed point) - 4.1V per cell
						lBatteryVoltage = msg.measurement.lBatteryVoltage;
						printf("Battery U: %2u.%01uV\r\n", lBatteryVoltage / 10, lBatteryVoltage % 10);
						if (lBatteryVoltage < 237) /* below 23.7V - switch off to protect battery! */
						{
							/* issue shutdown to self */
							xPowerMgmtMsg msg;
							msg.xType = COMMAND_SHUTDOWN;
							msg.shutdown.xDelay = xDelay100;
							printf("Requesting shutdown to protect battery!\r\n");
							xQueueSend(xPowerMgmtMsgQueue, &msg, (TickType_t)0);
						}
						else if (lBatteryVoltage > 287) /* over 28.7V - stop charging! */
						{
						}
					}
					if (msg.measurement.lBatteryTemperature != INT32_MIN)
					{
						lBatteryTemperature = msg.measurement.lBatteryTemperature;
						printf("Battery T: %3u.%01u%cC\r\n", lBatteryTemperature / 10, lBatteryTemperature % 10, '\xB0');
						if (lBatteryTemperature > 400) /* above 40.0°C - switch off to protect battery! */
						{
							/* issue shutdown to self */
							xPowerMgmtMsg msg;
							msg.xType = COMMAND_SHUTDOWN;
							msg.shutdown.xDelay = xDelay100;
							printf("Requesting shutdown to protect battery!\r\n");
							xQueueSend(xPowerMgmtMsgQueue, &msg, (TickType_t)0);
						}
					}
					break;
				}
				case (COMMAND_SHUTDOWN):
				{
					printf("Shutting down!\r\n");
					vTaskDelay(msg.shutdown.xDelay);
					LPC_GPIO1->FIOPIN &= ~(PIN(25)); // Shutdown
					__disable_irq(); for( ;; );
				}
				default:
					printf("Unknown msg\r\n");
			}
		}
	}
}
