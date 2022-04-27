//#define DEBUGCONSOLE 1
//#define debugPrintf 1
#define LPC177x_8x // to make visual studio understand the code better

#define LOWSTACKWARNING

#include <stdio.h>

#include "common.h"
#include "keypad.h"
#include "sensor.h"
#include "powermgmt.h"
//#include "motorctrl.h"
#include "lcd.h"
#include "redirect.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "global.h"

#ifdef LPC177x_8x // DB504
TODO("Cpu lpc1788 DB504")

HeapRegion_t xHeapRegions[] = {
    { (uint8_t *) 0x10001000UL, 0xF000 }, // 60kB
    { (uint8_t *) 0x20000000UL, 0x2000 }, // 8kB
    { (uint8_t *) 0x20002000UL, 0x2000 }, // 8kB
    { (uint8_t *) 0x20004000UL, 0x4000 }, // 16kB
    { NULL, 0 }                           // Tot: 92kB
};
#else // LPC1768 DB275
TODO("Cpu lpc1768 DB275")
#error "I fucked something up in LPC175x_6x_hal, check all pin setup, chkpin, pindir etc and also the special invert functions. \
(5x/6x does not have hw invert polarity!) \
Make a complete testsuite and test all those things. LCD does work and print though so it is probably something with reading a pin state. \
Something fishy going on.. :("
HeapRegion_t xHeapRegions[] = {
    { (uint8_t *) 0x10001000UL, 0x7000 }, // 28kB
    { (uint8_t *) 0x2007C000UL, 0x4000 }, // 16kB
    { (uint8_t *) 0x20080000UL, 0x4000 }, // 16kB
    { NULL, 0 }                           // Tot: 60kB
};
#endif

#if 0
static void task_DigitalTest(void *pvParameters)
{
    for (;;) {
        /*GPIO_SET_PIN(LCD_BACKLIGHT);
        vTaskDelay(xDelay500);
        GPIO_CLR_PIN(LCD_BACKLIGHT);*/
        vTaskDelay(xDelay500);
        printf("key: %i %i %i\r\n", keypad_GetKey(), keypad_GetState(), keypad_GetTime());

        /*        GPIO_SET_PIN(LCD_BACKLIGHT);
                vTaskDelay(xDelay200);
                GPIO_CLR_PIN(LCD_BACKLIGHT);
                vTaskDelay(xDelay200);*/
    }
}
#endif

int main(void) {
    vPortDefineHeapRegions(xHeapRegions);

    hardware_Init();
    MotorCtrl_Init();
    

    xSensorQueue = xQueueCreate(1, sizeof(xSensorMsgType));
    xSensorMsgType sensor = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
    xQueueOverwrite(xSensorQueue, &sensor);
    
/* https://freertos.org/a00125.html
     BaseType_t xTaskCreate(    TaskFunction_t pvTaskCode,
                            const char * const pcName,
                            configSTACK_DEPTH_TYPE usStackDepth,
                            void *pvParameters,
                            UBaseType_t uxPriority,
                            TaskHandle_t *pxCreatedTask
                          );
*/

    //xTaskCreate(task_DigitalTest, "Digital", 128, NULL, 5, NULL);
    xTaskCreate(keypad_Task, "Keypad", 150, NULL, 6, NULL); // configMINIMAL_STACK_SIZE
    xTaskCreate(LCD_Task, "LCD", 1024, NULL, 8, NULL);
    xTaskCreate(sensor_Task, "Sensor", 512, NULL, 5, NULL);
    xTaskCreate(powerMgmt_Task, "PowerMgmt", 160, NULL, 5, NULL);
    //xTaskCreate(task_MotorCtrl, "MotorCtrl", configMINIMAL_STACK_SIZE, NULL, 5, NULL);

    vTaskStartScheduler();
}
