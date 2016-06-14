#include "main.h"

/*
NOTE this is only a proof of concept. This code REALLY needs a lot of work! Some parts needs a total rewrite!
*/

void _ADC_IRQHandler(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    //LPC_ADC->ADCR &= ~((1 << 16) | (1 << 21)); // function? needed?

    NVIC_DisableIRQ(ADC_IRQn);
    NVIC_ClearPendingIRQ(ADC_IRQn);

    xSemaphoreGiveFromISR(binADCTask, &xHigherPriorityTaskWoken);

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

    return;
}

static void semADCTask(void *pvParameters)
{
    for (;;) {
        if (xSemaphoreTake(binADCTask, portMAX_DELAY) == pdTRUE) {
            ADC0 = (LPC_ADC->ADDR0 >> 4) & 4095;
            ADC1 = (LPC_ADC->ADDR1 >> 4) & 4095;
            ADC2 = (LPC_ADC->ADDR2 >> 4) & 4095;
            ADC3 = (LPC_ADC->ADDR3 >> 4) & 4095;
            ADC4 = (LPC_ADC->ADDR4 >> 4) & 4095;
            ADC5 = (LPC_ADC->ADDR5 >> 4) & 4095;
            ADC6 = (LPC_ADC->ADDR6 >> 4) & 4095;
            ADC7 = (LPC_ADC->ADDR7 >> 4) & 4095;
            debug3++;
            NVIC_EnableIRQ(ADC_IRQn);
            stack_ADC = uxTaskGetStackHighWaterMark(NULL);
        }

    }
}

void EINT3_IRQHandler(void)
{
    return;
}


void SystemSetupStuff(void)
{
    LPC_SC->PCONP |= PCONP_PCGPIO;              // power up GPIO
    LPC_GPIO1->FIODIR |= PIN(25);               // p1.25 Pwr on set output
    LPC_GPIO1->FIOPIN |= PIN(25);               // p1.25 Keep pwr on

    // Configure Timer0
    LPC_SC->PCONP |= PCONP_PCTIM1;              // power up Timer (def on)
    LPC_SC->PCLKSEL0 |= PCLK_TIMER1(CCLK_DIV1); // set Timer0 clock1

    // Configure LCD backligt
    LPC_GPIO1->FIODIR |= PIN(20);               // P1.20 output mode.
    LPC_GPIO1->FIOPIN |= PIN(20);               // p1.20 LCD backlight ON

    // Configure SPI (LCD)
    LPC_SC->PCONP |= PCONP_PCSPI;               // power up SPI
    LPC_SC->PCLKSEL0 |= PCLK_SPI(CCLK_DIV1);    // set SPI CCLK

    LPC_GPIO0->FIODIR |= PIN(rstb);             // p0.19 output mode.
    LPC_GPIO0->FIODIR |= PIN(csb);              // p0.16 output mode.
    LPC_GPIO0->FIODIR |= PIN(a0);               // p0.20 output mode.

    LPC_PINCON->PINSEL0 |= ((uint32_t)3 << 30); // p0.15 -> sck
    LPC_PINCON->PINSEL1 |= (0xc | 0x30);        // p0.17 & p0.18 miso / mosi (no miso??)

    LPC_SPI->SPCR |= SPCR_MSTR;                 // SPI operates in Master mode.

    // Configure Power button
    LPC_PINCON->PINMODE3 |= (PINMODE_PULLDOWN << 24);      //Pwr btn


    // External INT
    // TODO: Make extint_handler
    LPC_GPIOINT->IO0IntEnR = (1 << 7) | (1 << 9);
    LPC_GPIOINT->IO0IntEnF = (1 << 8) | (1 << 10);
    LPC_GPIOINT->IO0IntClr = (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10);


    //NVIC_SetPriority(21, 0);
    //NVIC_EnableIRQ(21);


    // Test quad switch? ?? TODO : What is function of quadswitch???
    /*
        LPC_GPIO0->FIODIR |= PIN(4);         // P0.4 output mode.
        LPC_GPIO0->FIODIR |= PIN(5);         // P0.5 output mode.
        LPC_GPIO0->FIOPIN |= PIN(4);         // selected // 5 = hardfault


        LPC_GPIO0->FIODIR |= PIN(21);         // P0.21 output mode.
        LPC_GPIO0->FIODIR |= PIN(22);         // P0.22 output mode.
        LPC_GPIO0->FIOPIN |= PIN(21);         // selected // 22 = hardfault

        LPC_GPIO0->FIODIR |= PIN(23);         // P0.23 output mode.
        LPC_GPIO0->FIODIR |= PIN(24);         // P0.24 output mode.
        LPC_GPIO0->FIOPIN |= PIN(24);         // selected
    */


    // Configure ADC
    LPC_SC->PCONP |= PCONP_PCADC;               // power up ADC

    LPC_SC->PCLKSEL0 |= PCLK_ADC(CCLK_DIV8);    // set ADC CCLK/8 = 15M

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

    //    LPC_ADC->ADINTEN = 0xff;                //all adc enable
    LPC_ADC->ADINTEN = 0x1;                //all adc enable

    LPC_ADC->ADCR &= ~((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3));
    LPC_ADC->ADCR |= 255 | (1 << 21);

    //NVIC_SetPriority(ADC_IRQn, 30);
    //NVIC_SetPriority(ADC_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    LPC_ADC->ADCR |= (1 << 16) | (1 << 21);  // Investigate!


    /*
     uint32_t ADC0 = 0; not in use??
     uint32_t ADC1 = 0;  volt batt? ~ 3600
     uint32_t ADC2 = 0;  tilt sideways
     uint32_t ADC3 = 0;  tilt forward
     uint32_t ADC4 = 0;   high = cold, low = warm?
     uint32_t ADC5 = 0;  current spindle?
     uint32_t ADC6 = 0;  current left?
     uint32_t ADC7 = 0;  current right?


     temp: 2540 (cold) (adc4) warmup = 2140
     volt: 27.36 = 3566
     volt: 26.74 = 3488


     ** from original firmware:
     val_volt_cal = 203;
     v16_batteryLowV = 237;
     v18_batteryFullCharge = 287;
     v19_detectBatteryFull = 0;


     val_volt_cal = 203;
     v16_batteryLowV = 237;
     v18_batteryFullCharge = 287;
     v19_detectBatteryFull = 0;
     v31_refVFor100 = 258;


     tempCalTbl:
     ROM:000246B0 F3 0C D6 0C+    tempCalTbl

     DCW  0xCF3, 0xCD6, 0xCB9, 0xC9B, 0xC7C, 0xC5A, 0xC3B, 0xC1B, 0xBFB, 0xBDB
     DCW  0xBB5, 0xB93, 0xB72, 0xB50, 0xB2E, 0xB01, 0xADE, 0xABA, 0xA97, 0xA73
     DCW  0xA46, 0xA23, 0x9FF, 0x9DB, 0x9B7, 0x984, 0x960, 0x93B, 0x917, 0x8F3
     DCW  0x8C1, 0x89E, 0x87A, 0x857, 0x834, 0x800, 0x7DD, 0x7BA, 0x798, 0x776
     DCW  0x743, 0x722, 0x701, 0x6E0, 0x6C0, 0x68E, 0x66F, 0x651, 0x633, 0x616
     DCW  0x5E4, 0x5C7, 0x5AB, 0x590, 0x575, 0x545, 0x52B, 0x511, 0x4F8, 0x4DF
     DCW  0x4B3, 0x49B, 0x484, 0x46E, 0x457, 0x42E, 0x419, 0x404, 0x3F0, 0x3DC
     DCW  0x3B6, 0x3A3, 0x391, 0x37F, 0x36D, 0x34A, 0x339, 0x329, 0x319, 0x30A
     DCW  0x2EA, 0x2DC, 0x2CD, 0x2C0, 0x2B2, 0x295, 0x288, 0x27C, 0x270, 0x264
     DCW  0x249, 0x23E, 0x234, 0x229, 0x21F, 0x207, 0x1FD, 0x1F3, 0x1EA, 0x1E1
     DCW  0x1CC, 0x1C4, 0x1BB, 0x1B3, 0x1AB, 0x198, 0x191, 0x18A, 0x183, 0x17C

     int __fastcall convert_temp(int raw_temp) {
         int _raw_temp; // r1@1
         int i1; // r0@1
         int i2; // r2@1

         _raw_temp = raw_temp;
         i1 = 0;
         i2 = 0;
         while (i1 != 110) {
             if (tempCalTbl[i1] < _raw_temp) {
                 if (i1 > 0) {
                     --i1;
                     for (i2 = 0; i2 != 10 && tempCalTbl[i1] - (tempCalTbl[i1] - tempCalTbl[i1 + 1]) * i2 / 10 > _raw_temp; ++i2)
                         ;
                 }
                 return i2 + 10 * (i1 - 10);
             }
             if (tempCalTbl[i1] == _raw_temp) {
                 i2 = 0;
                 return i2 + 10 * (i1 - 10);
             }
             ++i1;
         }
         return i2 + 10 * (i1 - 10);
     }

     */


    //TODO : Watchdog

    // Init keypad
    KeypadSetRow(0);
}

HeapRegion_t xHeapRegions[] = {
    { (uint8_t *) 0x10001000UL, 0x6fff },
    { (uint8_t *) 0x2007C000UL, 0x3fff },
    { (uint8_t *) 0x20080000UL, 0x3fff },
    { NULL, 0 }
};

int main(void)
{
    vPortDefineHeapRegions(xHeapRegions);
#ifdef debugSemohosting
    initialise_monitor_handles();
#endif


    //NVIC_SetPriorityGrouping(5); ??

    SystemCoreClockUpdate();
    SystemSetupStuff();
    LCDInit();

    // TODO: enable & read internal RTC

    portENTER_CRITICAL();
    NVIC_SetPriority(ADC_IRQn, 5);
    NVIC_EnableIRQ(ADC_IRQn);
    portEXIT_CRITICAL();

    vSemaphoreCreateBinary(binADCTask);
    xSemaphoreTake(binADCTask, 0);

    xQueue = xQueueCreate(mainQUEUE_LENGTH, sizeof(unsigned long));

    if (xQueue != NULL)  {
        xTaskCreate(task_CounterTest, "Counter", 1024, NULL, 5, NULL);
        xTaskCreate(task_Keypad, "Keypad", configMINIMAL_STACK_SIZE, NULL, 6, NULL);
        xTaskCreate(task_LCDUpdate, "LCD", 1024, NULL, 8, NULL);
        xTaskCreate(semADCTask, "semaphore", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
        vTaskStartScheduler();
    }
}


static void task_Keypad(void *pvParameters)
{
    for (;;) {
        vTaskDelay(xDelay100);
        keypadProcessTask();
        if (keypadGetKey() == KEYSTOP) {
            // Stop all PWM
            LPC_PWM1->MR1 = 0; // PWM1
            LPC_PWM1->MR2 = 0; // PWM2
            LPC_PWM1->MR3 = 0; // PWM3
            LPC_PWM1->LER = BIT(0) | BIT(1) | BIT(2) | BIT(3); // MR0 - MR3 enabled.

            // Disable all motors
            LPC_GPIO2->FIOPIN &= ~PIN(4); //R
            LPC_GPIO2->FIOPIN &= ~PIN(9);
            LPC_GPIO2->FIOPIN &= ~PIN(13);
            // Enable Brake on all motors
            LPC_GPIO2->FIOPIN |= PIN(5); //R
            LPC_GPIO2->FIOPIN |= PIN(8);
            LPC_GPIO3->FIOPIN |= PIN(25);
        }
        if (keypadGetKey() == KEYPWR && keypadGetTime() > 6) {
            // TODO: Shut down motor! break etc, draws current from battery even in cpu-off!
            // TODO: display shutdown counter! (doesn't shutdown until released (hardware feature))
            // CPU in sleep??
            LPC_GPIO1->FIOPIN &= ~(PIN(25)); // Shutdown
        }
        stack_Keypad = uxTaskGetStackHighWaterMark(NULL);
    }
}

static void task_CounterTest(void *pvParameters)
{
    int Counter1 = 0;
    for (;;) {
        debug4++;
        Counter1++;
        vTaskDelay(xDelay500);
        stack_Counter = uxTaskGetStackHighWaterMark(NULL);
#ifdef debugSemohosting
        if (keypadGetState() == 2) printf("State: %u Key: %u Time: %u\n", keypadGetState(), keypadGetKey(), keypadGetTime());
        if (Counter1 > 20) {
            printf("Heap left: LCD: %u Counter: %u Keypad: %u ADC: %u\n", stack_LCD, stack_Counter, stack_Keypad, stack_ADC);
            Counter1 = 0;
        }
#endif

    }
}

static void task_LCDUpdate(void *pvParameters)
{
    TickType_t xLastTime;
    xLastTime = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&xLastTime, xDelay100);
        lcdUpdate();
        debug3 = 0;
        stack_LCD = uxTaskGetStackHighWaterMark(NULL);
    }
}
