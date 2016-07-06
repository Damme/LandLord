#include <stdio.h>
#include "sensor.h"
#include "powermgmt.h"
#include "define.h"
#include "timers.h"
#include "event_groups.h"


#define xDelay10  ((TickType_t)10 / portTICK_PERIOD_MS)
#define xDelay100  ((TickType_t)100 / portTICK_PERIOD_MS)
#define xDelay1000  ((TickType_t)1000 / portTICK_PERIOD_MS)

#define BIT_ADC_DONE    ( 1 << 0 )
#define BIT_DIG_INT    ( 1 << 1 )


EventGroupHandle_t xSensorEventGroup;
xQueueHandle xBoundaryMsgQueue;
TimerHandle_t xADCTriggerTimer;

uint32_t ADC0;
uint32_t ADC1;
uint32_t ADC2;
uint32_t ADC3;
uint32_t ADC4;
uint32_t ADC5;
uint32_t ADC6;
uint32_t ADC7;

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

void vProcessDigitalInputs(void *pvParameter1, uint32_t ulParameter2)
{
    /* ...Perform the processing here... */

    /*
    bool sensorFront()
    {
        return LPC_GPIO4->FIOPIN & PIN(29);
    }
    bool sensorRain()
    {
        return LPC_GPIO1->FIOPIN & PIN(29);
    }
    bool sensorCover()
    {
        return LPC_GPIO4->FIOPIN & PIN(28);
    }
    bool sensorLift()
    {
        return LPC_GPIO1->FIOPIN & PIN(16);
    }
    bool sensorCharger()
    {
        return LPC_GPIO1->FIOPIN & PIN(21);
    }
    uint8_t sensorDIP()
    {
        uint8_t val = 0;
        val = ((LPC_GPIO2->FIOPIN >> 7) & 1) | (((LPC_GPIO2->FIOPIN >> 3) & 1) << 1) | (((LPC_GPIO0->FIOPIN >> 1) & 1) << 2);
        return val;
    }
    uint8_t sensorWireR()
    {
        uint8_t val = 0;
        val = ((LPC_GPIO0->FIOPIN >> 9) & 1) | (((LPC_GPIO0->FIOPIN >> 10) & 1) << 1);
        return val;
    }
    uint8_t sensorWireL()
    {
        uint8_t val = 0;
        val = ((LPC_GPIO0->FIOPIN >> 7) & 1) | (((LPC_GPIO0->FIOPIN >> 8) & 1) << 1);
        return val;
    }
    */
}

int32_t convertAcc(uint16_t adc);
void printAcc(int32_t acc);

void ADC_IRQHandler(void)
{
    volatile uint32_t gdr;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    LPC_ADC->ADCR &= ~(1 << 16);
    ADC0 = (LPC_ADC->ADDR0 >> 4) & 4095;
    ADC1 = (LPC_ADC->ADDR1 >> 4) & 4095;
    ADC2 = (LPC_ADC->ADDR2 >> 4) & 4095;
    ADC3 = (LPC_ADC->ADDR3 >> 4) & 4095;
    ADC4 = (LPC_ADC->ADDR4 >> 4) & 4095;
    ADC5 = (LPC_ADC->ADDR5 >> 4) & 4095;
    ADC6 = (LPC_ADC->ADDR6 >> 4) & 4095;
    ADC7 = (LPC_ADC->ADDR7 >> 4) & 4095;
    NVIC_ClearPendingIRQ(ADC_IRQn);

    xEventGroupSetBitsFromISR(xSensorEventGroup, BIT_ADC_DONE, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    return;
}


void EINT3_IRQHandler(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    NVIC_DisableIRQ(EINT3_IRQn);

    uint32_t timer = LPC_TIM2->TC; // Get µs counter
    if (timer == 0) LPC_TIM2->TCR = 1; // start if 0

    xQueueSendFromISR(xBoundaryMsgQueue, &timer, xHigherPriorityTaskWoken);

    // also need muxing p0.21 & p0.22 for distance and left/right -> outside int in task-sensor

    xEventGroupSetBitsFromISR(xSensorEventGroup, BIT_DIG_INT, &xHigherPriorityTaskWoken);

    // Important! If not cleared system freezes
    LPC_GPIOINT->IO0IntClr = PIN(7) | PIN(8) | PIN(9) | PIN(10);
    NVIC_ClearPendingIRQ(EINT3_IRQn);
    NVIC_EnableIRQ(EINT3_IRQn);



    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    return;
}

void vADCTriggerTimerCallback(TimerHandle_t xTimer)
{
    LPC_ADC->ADCR |= (1 << 16);
}

int32_t convert_temp(uint16_t raw_temp)
{
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

void task_Sensor(void *pvParameters)
{
    xSensorEventGroup = xEventGroupCreate();

    xBoundaryMsgQueue = xQueueCreate(20, sizeof(uint32_t));

    vTaskDelay(xDelay10);

    ADC0 = 0; // not in use??
    ADC1 = 0; // volt batt? ~ 3600
    ADC2 = 0; // tilt sideways
    ADC3 = 0; // tilt forward
    ADC4 = 0; // high = cold, low = warm?
    ADC5 = 0; // current spindle?
    ADC6 = 0; // current left?
    ADC7 = 0; // current right?

    // Configure ADC
    LPC_SC->PCONP |= PCONP_PCADC;               // power up ADC

    LPC_SC->PCLKSEL0 |= PCLK_ADC(CCLK_DIV8);    // set ADC CCLK/8 = 12.5M

    LPC_PINCON->PINSEL1 |= (1 << 14);       // p0.23 -> ad0.0
    LPC_PINCON->PINSEL1 |= (1 << 16);       // p0.24 -> ad0.1
    LPC_PINCON->PINSEL1 |= (1 << 18);       // p0.25 -> ad0.2
    LPC_PINCON->PINSEL1 |= (1 << 20);       // p0.26 -> ad0.3

    LPC_PINCON->PINSEL3 |= (1 << 28);       // p1.30 -> ad0.4
    LPC_PINCON->PINSEL3 |= (1 << 29);
    LPC_PINCON->PINSEL3 |= (1 << 30);       // p1.31 -> ad0.5
    LPC_PINCON->PINSEL3 |= ((uint32_t)1 << 31);
    LPC_PINCON->PINSEL0 |= (1 << 7);        // p0.3 -> ad0.6
    LPC_PINCON->PINSEL0 |= (1 << 5);        // p0.2 -> ad0.7

    LPC_ADC->ADINTEN = (1 << 7);              // done flag

    LPC_ADC->ADCR = 255 | (1 << 21);

    // Enable burst mode / Disable power-down
    //      LPC_ADC->ADCR |= (1 << 16) | (1 << 21);

    portENTER_CRITICAL();
    NVIC_SetPriority(ADC_IRQn, 5);
    NVIC_EnableIRQ(ADC_IRQn);
    portEXIT_CRITICAL();

    // External INT - only wire sensor P0.7-P0.10
    LPC_GPIOINT->IO0IntEnR = (1 << 7) | (1 << 9);
    LPC_GPIOINT->IO0IntEnF = (1 << 8) | (1 << 10);
    LPC_GPIOINT->IO0IntClr = (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10);

    // Configure Timer2 used for µs counter used in eint3 (wire)
    LPC_SC->PCONP |= PCONP_PCTIM2;              // power up Timer (def on)
    LPC_SC->PCLKSEL1 |= PCLK_TIMER2(CCLK_DIV1); // set Timer2 clock divider

    portENTER_CRITICAL();
    NVIC_SetPriority(EINT3_IRQn, 5);
    NVIC_EnableIRQ(EINT3_IRQn);
    portEXIT_CRITICAL();

    printf("Sensor interrupts enabled\r\n");

    // MUX switch - P0.4 + P0.5
    LPC_PINCON->PINSEL0 &= ~(3 << 8);
    LPC_PINCON->PINSEL0 &= ~(3 << 10);
    LPC_GPIO0->FIODIR |= (1 << 4) | (1 << 5);
    LPC_GPIO0->FIOSET |= (1 << 4) | (1 << 5);   // MUX=3 - Bat. Temp.

    xADCTriggerTimer = xTimerCreate("ADCTriggerTimer", xDelay1000 * 5, pdTRUE, (void *) 0, vADCTriggerTimerCallback);
    if ((xADCTriggerTimer != NULL) && (xTimerStart(xADCTriggerTimer, 0) == pdPASS))
        printf("ADC Trigger Timer started\r\n");
    else
        printf("ADC Trigger Timer failed\r\n");

    for (;;) {
        xPowerMgmtMsg msg;
        EventBits_t event = xEventGroupWaitBits(xSensorEventGroup, BIT_ADC_DONE | BIT_DIG_INT, pdTRUE, pdFALSE, xDelay100);
        if (event & BIT_ADC_DONE) {
            uint32_t ulMuxState;
            int32_t accx = convertAcc(ADC3);
            int32_t accy = convertAcc(ADC2);
            int32_t accz;

            //printf("\r\nADC: %04u %04u %04u %04u %04u %04u %04u %04u\r\n", ADC0, ADC1, ADC2, ADC3, ADC4, ADC5, ADC6, ADC7);

            //printf("Acc X (g): ");
            printAcc(accx);
            //printf("Acc Y (g): ");
            printAcc(accy);

            ulMuxState = (LPC_GPIO0->FIOPIN & ((1 << 4) | (1 << 5))) >> 4;
            //printf("Mux: %01u\r\n", ulMuxState);
            switch (ulMuxState) {
                case 0:
                    accx = convertAcc(ADC4);
                    printf("Acc X2 (g): ");
                    printAcc(accx);
                    break;
                case 1:
                    accy = convertAcc(ADC4);
                    printf("Acc Y2 (g): ");
                    printAcc(accy);
                    break;
                case 2:
                    accz = convertAcc(ADC4);
                    //printf("Acc Z (g): ");
                    printAcc(accz);
                    break;
            }
            /* iterate over all mux choices */
            //LPC_GPIO0->FIOPIN = (LPC_GPIO0->FIOPIN & ~((1<<4) | (1<<5))) | (((ulMuxState + 1) % 4) << 4);
            /* just toggle between choices 'b10' (Z-axis) and 'b11' (Bat. Temp.) */
            LPC_GPIO0->FIOPIN = (LPC_GPIO0->FIOPIN & ~(1 << 4)) | (1 << 5) | (((ulMuxState + 1) % 2) << 4);

            //printf("Spindle I: %04u\r\n", ADC5);
            //printf("Right motor I: %04u\r\n", ADC6);
            //printf("Left motor I: %04u\r\n", ADC7);

            msg.xType = MEASUREMENT_BATTERY;
            /* https://hackaday.io/project/6717-project-landlord/discussion-58892 */
            msg.measurement.lChargeCurrent = ADC0;
            msg.measurement.lBatteryVoltage = ADC1 * 1000 / 13018; /* 3770 = 28.9V, 28v = 3645, 27.5v = 3580, 27.36v = 3566, 26.74v = 3488 */
            if (ulMuxState == 3)
                msg.measurement.lBatteryTemperature = convert_temp(ADC4);
            else
                msg.measurement.lBatteryTemperature = INT32_MIN;
            xQueueSend(xPowerMgmtMsgQueue, &msg, (TickType_t)0);
        }
        if (event & BIT_DIG_INT) {
            printf("DIG %u\r\n", uxQueueMessagesWaiting(xBoundaryMsgQueue));
        }
#if LOWSTACKWARNING
        int stack = uxTaskGetStackHighWaterMark(NULL);
        if (stack < 50) printf("Task task_Sensor has %u words left in stack.\r\n", stack);
#endif
    }
}

int32_t convertAcc(uint16_t adc)
{
    return ((int32_t)adc - 2060) * 1000 / (2720 - 2060);
}

void printAcc(int32_t acc)
{
    if (acc >= 0) {
        printf(" %01d.%03u\r\n", acc / 1000, acc % 1000);
    } else {
        acc = ~acc;
        printf("-%01d.%03u\r\n", acc / 1000, acc % 1000);
    }
}
