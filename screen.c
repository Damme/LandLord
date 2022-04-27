#include "screen.h"
#include "common.h"
#include <stdio.h>
#include <string.h>
#include "global.h"

char buffer[50];

u8g_t u8g;

uint8_t lcdCounter;
const listItem_t mainMenuList[7];
const listItem_t spindleMenuList[7];

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
        GPIO_SET_PIN(MOTOR_MOSFET);
        GPIO_CLR_PIN(MOTOR_BLADE_ENABLE);
        GPIO_CLR_PIN(MOTOR_BLADE_BRAKE); // Release brakes!
        a1=200;
        LPC_PWM1->MR1 = a1;
	    LPC_PWM1->LER |= (1<<1);
    }
    if (keypad_GetKey() == KEY5 && !keypressed) {
        keypressed = true;
        GPIO_TGL_PIN(MOTOR_BLADE_ENABLE);
    }

    if (keypad_GetKey() == KEY6 && !keypressed) {
        keypressed = true;
        GPIO_TGL_PIN(MOTOR_BLADE_BRAKE);
    }
    if (keypad_GetKey() == KEY7 && !keypressed) {
        keypressed = true;
        GPIO_TGL_PIN(MOTOR_BLADE_FORWARD);
    }
    if (keypad_GetKey() == KEY8 && !keypressed) {
        keypressed = true;
        a1 = a1 + 100;
        if (a1 > 1000) a1 = 1000;
    	LPC_PWM1->MR1 = a1;
	    LPC_PWM1->LER |= (1<<1);
    }
    if (keypad_GetKey() == KEY9 && !keypressed) {
        keypressed = true;
        a1 = a1 - 100;
        if (a1 < 0) a1 = 0;
    	LPC_PWM1->MR1 = a1;
	    LPC_PWM1->LER |= (1<<1);
    }
    if (keypad_GetKey() == KEY0 && !keypressed) {
        keypressed = true;
        GPIO_TGL_PIN(MOTOR_MOSFET);
    }
}

void lcdPrintSensor() {
    xSensorMsgType sensor;
    xQueuePeek(xSensorQueue, &sensor, 0);

    u8g_SetDefaultBackgroundColor(&u8g);
    u8g_DrawBox(&u8g, 0, 0, 128, 64);
    u8g_SetDefaultForegroundColor(&u8g);
    u8g_SetFont(&u8g, u8g_font_4x6);
    //sprintf(buffer, "Front:%u Rain:%u Cover:%u Lift:%u", sensorFront(), sensorRain(), sensorCover(), sensorLift());
    //sprintf(buffer, "%ld %ld %ld", ADC[0], ADC[1], ADC[2]);
    //u8g_DrawStr(&u8g,  0, 10, buffer);
    
    //sprintf(buffer, "%ld %ld %ld", ADC[3], ADC[4], ADC[5]);
    //sprintf(buffer, "DIP:%u, Charger: %u", sensorDIP(), sensorCharger());
    //u8g_DrawStr(&u8g,  0, 20, buffer);
    
    //sprintf(buffer, "%ld %ld", ADC[6], ADC[7]);
    //sprintf(buffer, "SensorL:%u SensorR:%u", sensorWireL(), sensorWireR());
    //u8g_DrawStr(&u8g,  0, 30, buffer);
    
    sprintf(buffer, "Motor: %ldA %ldA %ldA", sensor.motorSCurrent, sensor.motorLCurrent, sensor.motorRCurrent);
    u8g_DrawStr(&u8g,  0, 30, buffer);
    
    sprintf(buffer, "Battery: %iC %imV %imA", sensor.batteryTemp, sensor.batteryVolt, sensor.batteryChargeCurrent);
    u8g_DrawStr(&u8g,  0, 50, buffer);


    /*sprintf(buffer, "0x%.8lx %ic %iv", TESTADC1, TESTADC2, (ADC[6]/131));
    u8g_DrawStr(&u8g,  0, 60, buffer);
*/

    if (keypad_GetKey() == KEY0 && !keypressed) {
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
    if (keypad_GetKey() == KEY5 && !keypressed) {
        keypressed = true;
    }
    if (keypad_GetKey() == KEY6 && !keypressed) {
        keypressed = true;
    }

}

const listItem_t mainMenuList[7] = {
    {"Spindle motor", &motortest, (uint8_t*)1},
    {"Left wheel", &motortest, (uint8_t*)2},
    {"Right wheel", &motortest, (uint8_t*)3},
    {"Sensors", &lcdPrintSensor, NULL},
    {"Debug io", &lcdPrintDebug, NULL},
    {"Childmenu test", NULL, &spindleMenuList},
    {NULL, NULL, NULL}
};

const listItem_t spindleMenuList[7] = {
    {"1", &testfunc, (uint8_t*)1},
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

currentDisp_t currentDisplay = {&M2, &lcdPrintDebug, NULL, 0}; // Startscreen

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
    // useful function! :P

    int enb, brk, dir = 0;
    int *enbPort, *brkPort, *dirPort, *pwmPort;

    char tmp[2] = "";

    if (keypad_GetKey() == KEYSTART && !keypressed) {
        keypressed = true;

        // test
        LPC_GPIO0->DIR |= PIN(11);
        LPC_GPIO1->DIR |= PIN(23);
    }

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
        pwmtest = 0;
        keypressed = true;
        *pwmPort = pwmtest;
        LPC_PWM1->LER = BIT(0) | BIT(1) | BIT(2) | BIT(3); // MR0 - MR3 enabled.
    }

    if (keypad_GetKey() == KEYUP && !keypressed) {
        pwmtest = pwmtest + 50;
        if (pwmtest > 1000) pwmtest = 1000;
        keypressed = true;
        *pwmPort = pwmtest;
        LPC_PWM1->LER = BIT(0) | BIT(1) | BIT(2) | BIT(3); // MR0 - MR3 enabled.
    }
    if (keypad_GetKey() == KEYDOWN && !keypressed) {
        pwmtest = pwmtest - 50;
        if (pwmtest < 0) pwmtest = 0;
        keypressed = true;
        *pwmPort = pwmtest;
        LPC_PWM1->LER = BIT(0) | BIT(1) | BIT(2) | BIT(3); // MR0 - MR3 enabled.
    }

    if (keypad_GetKey() == KEY1 && !keypressed) {
        keypressed = true;
        if (!a1) {
            a1 = 1;
            *enbPort |= enb;
        } else {
            a1 = 0;
            *enbPort &= ~enb;
        }

    }
    if (keypad_GetKey() == KEY2 && !keypressed) {
        keypressed = true;
        if (!a2) {
            a2 = 1;
            *brkPort |= brk;
        } else {
            a2 = 0;
            *brkPort &= ~brk;
        }

    }
    if (keypad_GetKey() == KEY3 && !keypressed) {
        keypressed = true;
        if (!a3) {
            a3 = 1;
            *dirPort |= dir;
        } else {
            *dirPort &= ~dir;
            a3 = 0;
        }
    }


    if (keypad_GetKey() == KEY4 && !keypressed) {
        keypressed = true;
        if (!a4) {
            a4 = 1;
            LPC_GPIO0->PIN |= PIN(11);
        } else {
            a4 = 0;
            LPC_GPIO0->PIN &= ~PIN(11);
        }
    }
    if (keypad_GetKey() == KEY5 && !keypressed) {
        keypressed = true;
        if (!a5) {
            a5 = 1;
            LPC_GPIO1->PIN |= PIN(23);
        } else {
            a5 = 0;
            LPC_GPIO1->PIN &= ~PIN(23);
        }
    }
    if (keypad_GetKey() == KEY6 && !keypressed) {
        keypressed = true;
        if (!a6) {
            a6 = 1;
        } else {
            a6 = 0;
        }
    }

    if (keypad_GetKey() == KEY7 && !keypressed) {
        keypressed = true;
        if (!a7) {
            a7 = 1;
        } else {
            a7 = 0;
        }
    }
    if (keypad_GetKey() == KEY8 && !keypressed) {
        keypressed = true;
        if (!a7) {
            a8 = 1;
        } else {
            a8 = 0;
        }
    }
    if (keypad_GetKey() == KEY9 && !keypressed) {
        keypressed = true;
        if (!a9) {
            a9 = 1;
        } else {
            a9 = 0;
        }
    }

    u8g_SetFont(&u8g, u8g_font_4x6);
    sprintf(buffer, "pwm:%u a1:%u a2:%u a3:%u a4:%u", pwmtest, a1, a2, a3, a4);
    u8g_DrawStr(&u8g,  0, (8 * 2), buffer);

    tmp[1] = 0x0;
    tmp[0] = ((LPC_GPIO0->PIN >> 10) & 1) ? 0x31 : 0x30;
    sprintf(buffer, "p0.10: %s", tmp);
    tmp[0] = ((LPC_GPIO0->PIN >> 27) & 1) ? 0x31 : 0x30;
    sprintf(buffer + strlen(buffer), " - p0.27: %s", tmp);
    u8g_DrawStr(&u8g,  0, (8 * 3), buffer);

    tmp[0] = ((LPC_GPIO0->PIN >> 7) & 1) ? 0x31 : 0x30;
    sprintf(buffer, "p0.7: %s", tmp);
    tmp[0] = ((LPC_GPIO0->PIN >> 8) & 1) ? 0x31 : 0x30;
    sprintf(buffer + strlen(buffer), " - p0.8: %s", tmp);
    tmp[0] = ((LPC_GPIO0->PIN >> 9) & 1) ? 0x31 : 0x30;
    sprintf(buffer + strlen(buffer), " - p0.9 %s", tmp);
    u8g_DrawStr(&u8g,  0, (8 * 4), buffer);

    tmp[0] = (a1 ? 0x31 : 0x30);
    sprintf(buffer, "Brake(1): %s", tmp);
    tmp[0] = (a2 ? 0x31 : 0x30);
    sprintf(buffer, "Enable(2): %s", tmp);
    tmp[0] = (a3 ? 0x31 : 0x30);
    sprintf(buffer + strlen(buffer), " - F/R(3): %s", tmp);
    u8g_DrawStr(&u8g,  0, (8 * 5), buffer);
    /*
        sprintf(buffer, "0:%04u ", ADC0);
        sprintf(buffer + strlen(buffer), "1:%04u ", ADC1);
        sprintf(buffer + strlen(buffer), "2:%04u ", ADC2);
        sprintf(buffer + strlen(buffer), "3:%04u", ADC3);
        u8g_DrawStr(&u8g,  0, (8 * 7), buffer);

        sprintf(buffer, "4:%04u ", ADC4);
        sprintf(buffer + strlen(buffer), "5:%04u ", ADC5);
        sprintf(buffer + strlen(buffer), "6:%04u ", ADC6);
        sprintf(buffer + strlen(buffer), "7:%04u", ADC7);
        u8g_DrawStr(&u8g,  0, (8 * 8), buffer);
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

void LCD_Update(void) {
    uint8_t w;
    /*
    TODO: Check if screen really needs update!!!
    */
    const menuItem_t *curMenu;
    u8g_FirstPage(&u8g);
    do {
        //https://github.com/olikraus/u8glib/wiki/fontgroupx11
        u8g_SetFont(&u8g, u8g_font_6x13B);
        u8g_SetFontPosTop(&u8g);
        curMenu = currentDisplay.selected;
        w = u8g_GetStrWidth(&u8g, curMenu->itemName);
        w = 64 - (w / 2);
        u8g_DrawStr(&u8g,  w, 0, curMenu->itemName);
        u8g_SetFont(&u8g, u8g_font_5x8);

        if (currentDisplay.command == NULL) {
            currentDisplay.command = curMenu->command;
            //    printf("Command set : %s", curMenu->itemName);
        }
        if (currentDisplay.parm == NULL) {
            currentDisplay.parm = curMenu->parm;
        }

        currentDisplay.command(currentDisplay.parm);


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
