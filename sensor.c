#include <stdio.h>
#include "common.h"
#include "sensor.h"
#include "powermgmt.h"
#include "common.h"
#include "timers.h"
#include "event_groups.h"
#include "global.h"

//#define BIT_ADC_DONE    ( 1 << 0 )
//#define BIT_DIG_INT    ( 1 << 1 )

EventGroupHandle_t xSensorEventGroup;
xQueueHandle xDIGMsgQueue;
TimerHandle_t xADCTriggerTimer;


// Rewrite temperature conversion to something better.. This is just wierd.
const uint16_t tempCalTbl[110] = {
    0xCF3, 0xCD6, 0xCB9, 0xC9B, 0xC7C, 0xC5A, 0xC3B, 0xC1B, 0xBFB, 0xBDB,
    0xBB5, 0xB93, 0xB72, 0xB50, 0xB2E, 0xB01, 0xADE, 0xABA, 0xA97, 0xA73,
    0xA46, 0xA23, 0x9FF, 0x9DB, 0x9B7, 0x984, 0x960, 0x93B, 0x917, 0x8F3,
    0x8C1, 0x89E, 0x87A, 0x857, 0x834, 0x800, 0x7DD, 0x7BA, 0x798, 0x776,
    0x743, 0x722, 0x701, 0x6E0, 0x6C0, 0x68E, 0x66F, 0x651, 0x633, 0x616,
    0x5E4, 0x5C7, 0x5AB, 0x590, 0x575, 0x545, 0x52B, 0x511, 0x4F8, 0x4DF,
    0x4B3, 0x49B, 0x484, 0x46E, 0x457, 0x42E, 0x419, 0x404, 0x3F0, 0x3DC,
    0x3B6, 0x3A3, 0x391, 0x37F, 0x36D, 0x34A, 0x339, 0x329, 0x319, 0x30A,
    0x2EA, 0x2DC, 0x2CD, 0x2C0, 0x2B2, 0x295, 0x288, 0x27C, 0x270, 0x264,
    0x249, 0x23E, 0x234, 0x229, 0x21F, 0x207, 0x1FD, 0x1F3, 0x1EA, 0x1E1,
    0x1CC, 0x1C4, 0x1BB, 0x1B3, 0x1AB, 0x198, 0x191, 0x18A, 0x183, 0x17C,
};

int32_t convert_temp(uint16_t raw_temp) {
    /* table holds 110 entries from -10°C to 100°C in full °C */
    int32_t fullDegree; /* full degrees */
    int32_t tenthDegree; /* 1/10th-degrees */
    for (fullDegree = 0; fullDegree < (sizeof(tempCalTbl) / sizeof(uint16_t)); fullDegree++) {
        if (tempCalTbl[fullDegree] < raw_temp) { /* just one above */
            if (fullDegree > 0) { /* valid range? */
                --fullDegree; /* one back */
                for (tenthDegree = 0; (tenthDegree < 10); tenthDegree++) { /* search along gradient */
                    if ((tempCalTbl[fullDegree] - (tempCalTbl[fullDegree] - tempCalTbl[fullDegree + 1]) * tenthDegree / 10) <= raw_temp)
                        break;
                }
            }
            goto out;
        }
        if (tempCalTbl[fullDegree] == raw_temp) {
            tenthDegree = 0;
            goto out;
        }
    }
out:
    return tenthDegree + 10 * (fullDegree - 10);
}

/*
int32_t convertAcc(uint16_t adc);
void printAcc(int32_t acc);
*/
static volatile uint8_t ADC_Interrupt_Done_Flag;

/*
void ADC_IRQHandler(void) { // DB275 uses IRQ try to use BURST mode as DB504
    //portENTER_CRITICAL();
    NVIC_DisableIRQ(ADC_IRQn);
    //portEXIT_CRITICAL();
    //LPC_ADC->INTEN &= ~((1 << ADC_Channel)); // channel

    ADC_Interrupt_Done_Flag = 1;
}
*/

// DB275 uses interrupt to read front sensor, need work and more testing.
void EINT3_IRQHandler(void) {
/*#ifndef LPC177x_8x // DB275
    uint32_t timer = LPC_TIM2->TC; // Get µs counter
    NVIC_DisableIRQ(EINT3_IRQn);

    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    if (timer == 0) LPC_TIM2->TCR = 1; // start if 0

    xQueueSendFromISR(xDIGMsgQueue, &timer, &xHigherPriorityTaskWoken);

    // also need muxing p0.21 & p0.22 for distance and left/right -> outside int in task-sensor

    xEventGroupSetBitsFromISR(xSensorEventGroup, BIT_DIG_INT, &xHigherPriorityTaskWoken);

    // Important! If not cleared system freezes
    LPC_GPIOINT->IO0IntClr = PIN(7) | PIN(8) | PIN(9) | PIN(10);
    NVIC_ClearPendingIRQ(EINT3_IRQn);
    NVIC_EnableIRQ(EINT3_IRQn);

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
#endif*/
    return;
}

void sensor_Task(void *pvParameters) {
    sensor_Init();
    xPowerMgmtMsg msg;
    xSensorMsgType sensor;

    for (;;) {
        vTaskDelay(xDelay200);
        if (xQueuePeek(xSensorQueue, &sensor, 0) == pdTRUE) {

/*            for (int i = 0; i <= 7; i++) {
                ADC[i]=ADC_DR_RESULT(LPC_ADC->DR[i]);
            }
*/

            // handle_ADCMuxing(); // For LPC1768! ADC4 is muxed between 4 measurements.        
            
            sensor.batteryTemp = convert_temp(ADC_DR_RESULT(ANALOG_BATT_TEMP));
            sensor.batteryChargeCurrent = ADC_DR_RESULT(ANALOG_BATT_CHARGE_A);
            sensor.batteryVolt = ADC_DR_RESULT(ANALOG_BATT_VOLT) * 100000 / 13068;
            sensor.motorRCurrent = ADC_DR_RESULT(ANALOG_MOTOR_R_AMP);
            sensor.motorLCurrent = ADC_DR_RESULT(ANALOG_MOTOR_L_AMP);
            sensor.motorSCurrent = ADC_DR_RESULT(ANALOG_MOTOR_S_AMP);
            sensor.rainAnalog = ADC_DR_RESULT(ANALOG_RAIN);
            
            xQueueOverwrite(xSensorQueue, &sensor);

            /* https://hackaday.io/project/6717-project-landlord/discussion-58892 */
            msg.xType = MEASUREMENT_BATTERY;
            msg.measurement.lChargeCurrent = sensor.batteryChargeCurrent;
            msg.measurement.lBatteryVoltage = sensor.batteryVolt;
            msg.measurement.lBatteryTemperature = sensor.batteryTemp;
            xQueueSend(xPowerMgmtMsgQueue, &msg, (TickType_t)0);
        }
    }
}

