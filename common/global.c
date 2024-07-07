#include <sys/stat.h>
#include "common.h"
#include "global.h"

//int stack_Keypad, stack_Counter, stack_LCD, stack_ADC = 0;

TickType_t xDelay1 = TicksPerMS * 1;
TickType_t xDelay5 = TicksPerMS * 5;
TickType_t xDelay10 = TicksPerMS * 10;
TickType_t xDelay25 = TicksPerMS * 25;
TickType_t xDelay50 = TicksPerMS * 50;
TickType_t xDelay100 = TicksPerMS * 100;
TickType_t xDelay200 = TicksPerMS * 200;
TickType_t xDelay150 = TicksPerMS * 150;
TickType_t xDelay250 = TicksPerMS * 250;
TickType_t xDelay500 = TicksPerMS * 500;
TickType_t xDelay1000 = TicksPerMS * 1000;
TickType_t xDelay2000 = TicksPerMS * 2000;

volatile uint32_t cpuID = 0;

volatile TaskHandle_t xHandle[15] = {[0 ... 14] = NULL};
volatile uint8_t taskcounter = 0;

xQueueHandle xScreenMsgQueue;
xQueueHandle xMotorMsgQueue;
xQueueHandle xBoundaryMsgQueue;
xQueueHandle xJSONMessageQueue;

SensorType sensorMsg;

MessageBufferHandle_t SPI0RxMessageBuffer;
MessageBufferHandle_t SPI0TxMessageBuffer;
xQueueHandle RosTxQueue;

void wdt_reset() {
    // Feed sequence to reset the WDT
    LPC_WDT->FEED = 0xAA;
    LPC_WDT->FEED = 0x55;
}

void wdt_init() {
    // Enable the clock to the WDT
    LPC_SC->PCONP |= (1 << 15); 
    // Set reset to 4Seconds
    LPC_WDT->TC = 500000;
    // Set the mode register to enable reset, and then enable the watchdog timer
    LPC_WDT->MOD = (1 << 0) | (1 << 1);
    // Init by reset
    wdt_reset();
}

void wdt_disable() {
    // Disable the watchdog timer
    LPC_WDT->MOD &= ~((1 << 0) | (1 << 1));  // Clear both WDEN and WDRESET bits
}

void delay_uS(uint32_t uS) {
    portENTER_CRITICAL();
    LPC_TIM1->TCR = 0x02;                // reset timer
    LPC_TIM1->PR  = 0x00;                // set prescaler to zero
    LPC_TIM1->MR0 = uS * (SystemCoreClock / 1000000) - 1;
    //LPC_TIM1->IR  = 0xff;                // reset all interrrupts
    LPC_TIM1->MCR = 0x04;                // stop timer on match
    LPC_TIM1->TCR = 0x01;                // start timer
    // wait until delay time has elapsed
    while (LPC_TIM1->TCR & 0x01);
    portEXIT_CRITICAL();
}

void vAssertCalled( void ) {
    volatile unsigned long looping = 0;
    GPIO_TGL_PIN(LCD_BACKLIGHT);

    taskENTER_CRITICAL();
    {
        /* Use the debugger to set ul to a non-zero value in order to step out
         *      of this function to determine why it was called. */
        while( looping == 0LU ) {
            portNOP();
        }
    }
    taskEXIT_CRITICAL();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    for (;;);
}


void vApplicationMallocFailedHook(void) {
    for (;;);
}

int _close_r(struct _reent *r, int file) {
    return 0;
}

int _fstat_r(struct _reent *r, int file, struct stat *st) {
    return 0;
}

pid_t _getpid_r(struct _reent *r) {
    return 0;
}

int _isatty_r(struct _reent *r, int file) {
    return 0;
}

int _kill_r(struct _reent *r, int pid, int sig) {
    return 0;
}

off_t _lseek_r(struct _reent *r, int file, off_t ptr, int dir) {
    return 0;
}

ssize_t _read_r(struct _reent *r, int file, void *ptr, size_t len) {
    return 0;
}
/*
typedef void (*IAP)(uint32_t[5], uint32_t[5]);
const IAP IAP_entry = (IAP)0x1FFF1FF1;

int iap_read_id(void)
{
    unsigned int cmd[5], res[5];
    cmd[0] = 54;
    IAP_entry(cmd, res);
    return ((int)res[1]);
}
*/
