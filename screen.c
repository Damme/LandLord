#include "screen.h"
#include "common.h"
#include <stdio.h>
#include <string.h>
#include "global.h"

char buffer[50];

u8g_t u8g;

uint8_t lcdCounter;
const listItem_t mainMenuList[7];
const listItem_t menuSublist1[7];

const menuItem_t M0;
const menuItem_t M1;
const menuItem_t M2;

uint32_t counter = 0;

currentDisp_t currentDisplay;

int pwmtest, a1, a2, a3, a4, a5, a6, a7, a8, a9 = 0;
bool keypressed = false; // move to keypad and implement repeat key?

void LCD_Init(void) {
    u8g_InitComFn(&u8g, &u8g_dev_st7565_nhd_c12864_2x_hw_spi, u8g_com_hw_spi_fn);
    u8g_SetContrast(&u8g, LCD_CONTRAST);
    u8g_SetDefaultBackgroundColor(&u8g);
    u8g_SetRot180(&u8g);
    u8g_FirstPage(&u8g);
    do {
    } while (u8g_NextPage(&u8g));
    u8g_SetDefaultForegroundColor(&u8g);
}

void lcdPrintDebug() {
    uint8_t i;
    u8g_SetDefaultBackgroundColor(&u8g);
    u8g_DrawBox(&u8g, 0, 0, 128, 64);
    u8g_SetDefaultForegroundColor(&u8g);
    u8g_SetFont(&u8g, u8g_font_4x6);

    for (i = 0 ; i < 32 ; i++) {
        if ((LPC_GPIO0->PIN >> i) & 1) {
            buffer[i] = 0x31;
        } else {
            buffer[i] = 0x2e;
        }
    }
    buffer[32] = 0x0;
    u8g_DrawStr(&u8g,  0, 10, buffer);
    for (i = 0 ; i < 32 ; i++) {
        if ((LPC_GPIO1->PIN >> i) & 1) {
            buffer[i] = 0x31;
        } else {
            buffer[i] = 0x2e;
        }
    }
    buffer[32] = 0x0;
    u8g_DrawStr(&u8g,  0, 20, buffer);
    for (i = 0 ; i < 32 ; i++) {
        if ((LPC_GPIO2->PIN >> i) & 1) {
            buffer[i] = 0x31;
        } else {
            buffer[i] = 0x2e;
        }
    }
    buffer[32] = 0x0;
    u8g_DrawStr(&u8g,  0, 30, buffer);
    for (i = 0 ; i < 32 ; i++) {
        if ((LPC_GPIO3->PIN >> i) & 1) {
            buffer[i] = 0x31;
        } else {
            buffer[i] = 0x2e;
        }
    }
    buffer[32] = 0x0;
    u8g_DrawStr(&u8g,  0, 40, buffer);
    for (i = 0 ; i < 32 ; i++) {
        if ((LPC_GPIO4->PIN >> i) & 1) {
            buffer[i] = 0x31;
        } else {
            buffer[i] = 0x2e;
        }
    }
    buffer[32] = 0x0;
#ifdef LPC177x_8x // DB504
    u8g_DrawStr(&u8g,  0, 50, buffer);
    for (i = 0 ; i < 4 ; i++) {
        if ((LPC_GPIO5->PIN >> i) & 1) {
            buffer[i] = 0x31;
        } else {
            buffer[i] = 0x2e;
        }
    }
    buffer[5] = 0x0;
    u8g_DrawStr(&u8g,  0, 60, buffer);
#endif

    if (keypad_GetKey() == KEY1 && !keypressed) {
        keypressed = true;
        GPIO_PIN_FNC(LCD_BACKLIGHT_PWM);
    }

    if (keypad_GetKey() == KEY2 && !keypressed) {
        keypressed = true;
        pwmtest = pwmtest + 100;
        if (pwmtest > 1000) pwmtest = 1000;
    	LPC_PWM1->MR2 = pwmtest;
	    LPC_PWM1->LER |= (1<<2);
    }

    if (keypad_GetKey() == KEY3 && !keypressed) {
        keypressed = true;
        pwmtest = pwmtest - 100;
        if (pwmtest < 0) pwmtest = 0;
    	LPC_PWM1->MR2 = pwmtest;
	    LPC_PWM1->LER |= (1<<2);
    }
    
    if (keypad_GetKey() == KEY4 && !keypressed) {
        keypressed = true;
    }
    if (keypad_GetKey() == KEY5 && !keypressed) {
        keypressed = true;
    }

    if (keypad_GetKey() == KEY6 && !keypressed) {
        keypressed = true;
    }
    if (keypad_GetKey() == KEY7 && !keypressed) {
        keypressed = true;
    }
    if (keypad_GetKey() == KEY8 && !keypressed) {
        keypressed = true;
    }
    if (keypad_GetKey() == KEY9 && !keypressed) {
        keypressed = true;
    }
    if (keypad_GetKey() == KEY0 && !keypressed) {
        keypressed = true;
    }
    if (keypad_GetKey() == KEYHOME && !keypressed) {
        keypressed = true;
    }
}


void lcdPrintSensor() {
    xSensorMsgType sensor;
    xQueuePeek(xSensorQueue, &sensor, 0);

    //u8g_SetDefaultBackgroundColor(&u8g);
    //u8g_DrawBox(&u8g, 0, 0, 128, 64);
    u8g_SetDefaultForegroundColor(&u8g);
    u8g_SetFont(&u8g, u8g_font_4x6);
    
    sprintf(buffer, "Battery: %iC %imV %imA", sensor.batteryTemp, sensor.batteryVolt, sensor.batteryChargeCurrent);
    u8g_DrawStr(&u8g,  0, 20, buffer);

    sprintf(buffer, "T: %iC Rain %i C%iL%iS1%iS2%iD%i", sensor.boardTemp, sensor.rainAnalog, sensor.collision, sensor.lift, sensor.stuck, sensor.stuck2, sensor.door);
    u8g_DrawStr(&u8g,  0, 28, buffer);

    sprintf(buffer, "Motor (RPM??): %ld %ld %ld", sensor.motorSCurrent, sensor.motorLCurrent, sensor.motorRCurrent);
    u8g_DrawStr(&u8g,  0, 36, buffer);

    sprintf(buffer, "Accel XYZ: %+04d %+04d %+04d", sensor.AccelX, sensor.AccelY, sensor.AccelZ);
    u8g_DrawStr(&u8g,  0, 44, buffer);
    
    sprintf(buffer, "Ya Pi Ro: %+06d %+06d %+06d", sensor.MotionYaw, sensor.MotionPitch, sensor.MotionRoll);
    u8g_DrawStr(&u8g,  0, 52, buffer);    


    if (keypad_GetKey() == KEY1 && !keypressed) {
        keypressed = true;
    }
    if (keypad_GetKey() == KEY2 && !keypressed) {
        keypressed = true;
    }

    if (keypad_GetKey() == KEY3 && !keypressed) {
        keypressed = true;
    }

    if (keypad_GetKey() == KEY4 && !keypressed) {
        keypressed = true;
    }
    if (keypad_GetKey() == KEY5 && !keypressed) {
        keypressed = true;
    }
    if (keypad_GetKey() == KEY6 && !keypressed) {
        keypressed = true;
    }

}

void taskRuntimeStats() {
    u8g_SetDefaultForegroundColor(&u8g);
    u8g_SetFont(&u8g, u8g_font_4x6);
   /* TaskStatus_t *pxTaskStatusArray;
    volatile UBaseType_t uxArraySize, x;
    unsigned long ulTotalRunTime, ulStatsAsPercentage;
    uint8_t line = 14;

   // Take a snapshot of the number of tasks in case it changes while this function is executing. 
   uxArraySize = uxTaskGetNumberOfTasks();

   // Allocate a TaskStatus_t structure for each task.  An array could be allocated statically at compile time. 
   pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

   if( pxTaskStatusArray != NULL )
   {
      // Generate raw status information about each task. 
      uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
                                 uxArraySize,
                                 &ulTotalRunTime );

      // For percentage calculations. 
      ulTotalRunTime /= 100UL;

      // Avoid divide by zero errors. 
      if( ulTotalRunTime > 0 )
      {
         // For each populated position in the pxTaskStatusArray array, format the raw data as human readable ASCII data. 
         for( x = 0; x < uxArraySize; x++ )
         {
            // What percentage of the total run time has the task used? This will always be rounded down to the nearest integer.
            // ulTotalRunTimeDiv100 has already been divided by 100.
            ulStatsAsPercentage =
                  pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime;

            if( ulStatsAsPercentage > 0UL )
            {
               sprintf( buffer, "%i: %stt%lutt%lu%%", x,
                                 pxTaskStatusArray[ x ].pcTaskName,
                                 pxTaskStatusArray[ x ].ulRunTimeCounter,
                                 ulStatsAsPercentage );
            }
            else
            {
               // If the percentage is zero here then the task has consumed less than 1% of the total run time. 
               sprintf( buffer, "%i: %stt%lutt<1%%", x,
                                 pxTaskStatusArray[ x ].pcTaskName,
                                 pxTaskStatusArray[ x ].ulRunTimeCounter );
            }

            //pcWriteBuffer += strlen( ( char * ) pcWriteBuffer );
            u8g_DrawStr(&u8g,  0, line, buffer);
            line+=8;
         }
      }

      // The array is no longer needed, free the memory it consumes. 
      vPortFree( pxTaskStatusArray );
   }*/

}

//    12345678901234567890123456789012
const listItem_t mainMenuList[7] = {
    {"Spindle motor", &motortest, (uint8_t*)1},
    {"Left wheel", &motortest, (uint8_t*)2},
    {"Right wheel", &motortest, (uint8_t*)3},
    {"Sensors", &lcdPrintSensor, NULL},
    {"Debug io", &lcdPrintDebug, NULL},
    {"Childmenu test", NULL, &menuSublist1},
    {NULL, NULL, NULL}
};

const listItem_t menuSublist1[7] = {
    {"Task Runtime Stats", &taskRuntimeStats, (uint8_t*)1},
    {"2", &testfunc, (uint8_t*)2},
    {"3", &testfunc, (uint8_t*)3},
    {"4", &testfunc, (uint8_t*)4},
    {"5", &testfunc, (uint8_t*)5},
    {"6", &testfunc, (uint8_t*)6},
    {NULL, NULL, NULL}
};

const menuItem_t M0 = {/*0, SCREEN,*/ "Boot", menuBootfnc, NULL};
const menuItem_t M1 = {/*1, SCREEN,*/ "WARNING!", menuBootWarnfnc, NULL};
const menuItem_t M2 = {/*1, SCREEN,*/ "Main", menuListfnc, &mainMenuList};

//currentDisp_t currentDisplay = {&M0, NULL, NULL, 0}; // Startscreen with warning
currentDisp_t currentDisplay = {&M2, &lcdPrintSensor, NULL, 0}; // Startscreen

uint8_t lcdCounter = 0;


void menuListfnc(void *ptr) {
    uint8_t h, w, count;
    uint8_t *curItem;

    const listItem_t *thisItem = (listItem_t *) ptr;
    u8g_SetFontRefHeightExtendedText(&u8g);
    h = u8g_GetFontAscent(&u8g) - u8g_GetFontDescent(&u8g);
    w = u8g_GetWidth(&u8g);

    count = 0;
    curItem = &currentDisplay.value;

    if (keypad_GetKey() == KEYDOWN && !keypressed) {
        if ((thisItem + *curItem + 1)->itemName != NULL)(*curItem)++;
        keypressed = true;
    }
    if (keypad_GetKey() == KEYUP && !keypressed) {
        if (*curItem != 0)(*curItem)--;
        keypressed = true;
    }
    if (keypad_GetKey() == KEYOK && !keypressed) {
        if ((thisItem + *curItem)->command != NULL) {
            currentDisplay.command = (thisItem + *curItem)->command;
        }
        if ((thisItem + *curItem)->parm != NULL) {
            currentDisplay.parm = (thisItem + *curItem)->parm;
        }
        keypressed = true;
    }

    //Hold down for repeated press  || (keypressed && keypad_GetTime() % 2)

    while ((thisItem + count)->itemName != NULL) {
        u8g_SetDefaultForegroundColor(&u8g);
        if (*curItem == count) {
            u8g_DrawBox(&u8g, 0, (20 + h * (count - 1)), w, h + 1);
            u8g_SetDefaultBackgroundColor(&u8g);
        }
        u8g_DrawStr(&u8g,  0, (20 + h * count), (thisItem + count)->itemName);
        count++;
        u8g_SetDefaultForegroundColor(&u8g);
    }



}

void testfunc(void) {
    // useful function! :P
#ifdef debugPrintf
    printf("func! Parm: %u\r\n", (int)currentDisplay.parm); // check if null
#endif
    currentDisplay.command = NULL;
    currentDisplay.parm = NULL;
}



void motortest(void) {
    int enb, brk, dir = 0;
    int *enbPort, *brkPort, *dirPort, *pwmPort;

    char tmp[2] = "";
/*
    if (keypad_GetKey() == KEYSTART && !keypressed) {
        keypressed = true;

        // test
        LPC_GPIO0->DIR |= PIN(11);
        LPC_GPIO1->DIR |= PIN(23);
    }
// 1768 code not ok for 1788:
    if ((int)currentDisplay.parm == 1) {
        //Spindle
        enbPort = (int*) &LPC_GPIO2->PIN;
        enb = PIN(13);
        brkPort = (int*) &LPC_GPIO3->PIN;
        brk = PIN(25);
        dirPort = (int*) &LPC_GPIO3->PIN;
        dir = PIN(26);
        pwmPort = (int*) &LPC_PWM1->MR3;
    }
    if ((int)currentDisplay.parm == 2) {
        //Left
        enbPort = (int*) &LPC_GPIO2->PIN;
        enb = PIN(9);
        brkPort = (int*) &LPC_GPIO2->PIN;
        brk = PIN(8);
        dirPort = (int*) &LPC_GPIO0->PIN;
        dir = PIN(0);
        pwmPort = (int*) &LPC_PWM1->MR2;
    }
    if ((int)currentDisplay.parm == 3) {
        //Right
        enbPort = (int*) &LPC_GPIO2->PIN;
        enb = PIN(4);
        brkPort = (int*) &LPC_GPIO2->PIN;
        brk = PIN(5);
        dirPort = (int*) &LPC_GPIO2->PIN;
        dir = PIN(6);
        pwmPort = (int*) &LPC_PWM1->MR1;
    }


    if (keypad_GetKey() == KEY0 && !keypressed) {
        keypressed = true;
    }

    if (keypad_GetKey() == KEYUP && !keypressed) {
        keypressed = true;
    }
    if (keypad_GetKey() == KEYDOWN && !keypressed) {
        keypressed = true;
    }

    if (keypad_GetKey() == KEY1 && !keypressed) {
        keypressed = true;
    }
    if (keypad_GetKey() == KEY2 && !keypressed) {
        keypressed = true;
    }
    if (keypad_GetKey() == KEY3 && !keypressed) {
        keypressed = true;
    }
    if (keypad_GetKey() == KEY4 && !keypressed) {
        keypressed = true;
    }
    if (keypad_GetKey() == KEY5 && !keypressed) {
        keypressed = true;
    }
    if (keypad_GetKey() == KEY6 && !keypressed) {
        keypressed = true;
    }
    if (keypad_GetKey() == KEY7 && !keypressed) {
        keypressed = true;
    }
    if (keypad_GetKey() == KEY8 && !keypressed) {
        keypressed = true;
    }
    if (keypad_GetKey() == KEY9 && !keypressed) {
        keypressed = true;
    }
    */
}

void menuBootfnc(void) {
    lcdCounter++;
    if (lcdCounter > 100) {
        currentDisplay.selected = &M1;
        currentDisplay.command = NULL;
        currentDisplay.parm = NULL;
        lcdCounter = 0;
    }
}

void menuBootWarnfnc(void) {
    uint8_t h;
    //             12345678901234567890123456789012

    char *line1 = "  REMOVE KNIVES BEFORE RUNNING";
    char *line2 = "This firmware might cut";
    char *line3 = "all your fingers off, kill";
    char *line4 = "your cat or spontaneous combust!";
    char *line5 = "So please - Remove knives from";
    char *line6 = "spindle before continuing.";
    char *line7 = "   Hold down OK to contiune!";
    u8g_SetFont(&u8g, u8g_font_4x6);
    u8g_SetFontRefHeightExtendedText(&u8g);
    h = u8g_GetFontAscent(&u8g) - u8g_GetFontDescent(&u8g);

    u8g_DrawStr(&u8g,  0, (20 + h * 0), line1);
    u8g_DrawStr(&u8g,  0, (24 + h * 1), line2);
    u8g_DrawStr(&u8g,  0, (24 + h * 2), line3);
    u8g_DrawStr(&u8g,  0, (24 + h * 3), line4);
    u8g_DrawStr(&u8g,  0, (24 + h * 4), line5);
    u8g_DrawStr(&u8g,  0, (24 + h * 5), line6);
    u8g_DrawStr(&u8g,  0, (28 + h * 6), line7);
    if (keypad_GetKey() == KEYOK && keypad_GetTime() > 6) {
        currentDisplay.selected = &M2; // TODO next menu
        currentDisplay.command = NULL;
        currentDisplay.parm = NULL;
        keypressed = true;
    }

}

uint32_t override_timer = 0;

void screen_Task(void) {
    xScreenMsgType screenMsg;
    
    uint8_t w;

    const menuItem_t *curMenu;
    u8g_FirstPage(&u8g);
    if (override_timer > 0) override_timer--;

//    TODO: Check if screen really needs update?
    do {
        //https://github.com/olikraus/u8glib/wiki/fontgroupx11
// TODO: What is this? Title? Always prints "Main" on top of screen.
        u8g_SetFont(&u8g, u8g_font_6x13B);
        u8g_SetFontPosTop(&u8g);
        curMenu = currentDisplay.selected;
        w = u8g_GetStrWidth(&u8g, curMenu->itemName);
        w = 64 - (w / 2);
        u8g_DrawStr(&u8g,  w, 0, curMenu->itemName);
        u8g_SetFont(&u8g, u8g_font_5x8);
// end off title print

        if (currentDisplay.command == NULL) {
            currentDisplay.command = curMenu->command;
            //    printf("Command set : %s", curMenu->itemName);
        }
        if (currentDisplay.parm == NULL) {
            currentDisplay.parm = curMenu->parm;
        }

        if (override_timer == 0 ) {
            if (xQueueReceive(xScreenMsgQueue, &screenMsg, 0) == pdTRUE) {
                override_timer = screenMsg.time;
            } else {
                currentDisplay.command(currentDisplay.parm);
            }
        }
// Why does this only print for a short while? override_timer gets reset by it self???
        if (override_timer != 0 ) {
            u8g_SetDefaultBackgroundColor(&u8g);
            u8g_DrawBox(&u8g, 0, 0, 128, 64);
            u8g_SetDefaultForegroundColor(&u8g);
            u8g_SetFont(&u8g, u8g_font_4x6);
            w = u8g_GetStrWidth(&u8g, screenMsg.text);
            w = 64 - (w / 2);
            u8g_DrawStr(&u8g,  w, 30, screenMsg.text);
        }

        if (keypad_GetKey() == KEYBACK && !keypressed) {
            //TODO: prev scrn
            currentDisplay.parm = NULL;
            currentDisplay.command = NULL;
            keypressed = true;
        }
        if (!keypad_GetState()) {
            keypressed = false;
// TODO: resetKeyTime fnc keypad.c
        }


    } while (u8g_NextPage(&u8g));
}
