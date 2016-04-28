#include "screen.h"

void LCDInit(void)
{
    u8g_InitComFn(&u8g, &u8g_dev_st7565_nhd_c12864_2x_hw_spi, u8g_com_hw_spi_fn);
    u8g_SetContrast(&u8g, 4);
    u8g_SetDefaultBackgroundColor(&u8g);
    u8g_SetRot180(&u8g);
    u8g_FirstPage(&u8g);
    do {
    } while (u8g_NextPage(&u8g));
    u8g_SetDefaultForegroundColor(&u8g);
}

void lcdPrintDebug()
{
    u8g_SetDefaultBackgroundColor(&u8g);
    u8g_DrawBox(&u8g, 0, 0, 128, 64);
    u8g_SetDefaultForegroundColor(&u8g);
    u8g_SetFont(&u8g, u8g_font_4x6);
    sprintf(buffer, "Kdn:%u d1:%u d2:%u d3:%u d4:%u", keypadGetKey(), debug1, debug2, debug3, debug4);
    // binary debug
    u8g_DrawStr(&u8g,  0, 10, buffer);
    u8g_DrawStr(&u8g,  0, 20, "01234567890123456789012345678901");
    for (uint8_t i = 0 ; i < 32 ; i++) {
        if ((LPC_GPIO0->FIOPIN >> i) & 1) {
            buffer[i] = 0x31;
        } else {
            buffer[i] = 0x30;
        }
    }
    buffer[32] = 0x0;
    u8g_DrawStr(&u8g,  0, 30, buffer);
    for (uint8_t i = 0 ; i < 32 ; i++) {
        if ((LPC_GPIO1->FIOPIN >> i) & 1) {
            buffer[i] = 0x31;
        } else {
            buffer[i] = 0x30;
        }
    }
    buffer[32] = 0x0;
    u8g_DrawStr(&u8g,  0, 40, buffer);
    for (uint8_t i = 0 ; i < 32 ; i++) {
        if ((LPC_GPIO2->FIOPIN >> i) & 1) {
            buffer[i] = 0x31;
        } else {
            buffer[i] = 0x30;
        }
        if (i >= 14) buffer[i] = 0x5f;

        if (i == 20) buffer[i] = ((LPC_GPIO3->FIOPIN >> 25) & 1) ? 0x31 : 0x30;
        if (i == 21) buffer[i] = ((LPC_GPIO3->FIOPIN >> 26) & 1) ? 0x31 : 0x30;
        if (i == 23) buffer[i] = ((LPC_GPIO4->FIOPIN >> 28) & 1) ? 0x31 : 0x30;
        if (i == 24) buffer[i] = ((LPC_GPIO4->FIOPIN >> 29) & 1) ? 0x31 : 0x30;
    }
    buffer[32] = 0x0;
    u8g_DrawStr(&u8g,  0, 48, buffer);


    sprintf(buffer, "A");
    sprintf(buffer + strlen(buffer), "0:%04lu ", ADC0);
    sprintf(buffer + strlen(buffer), "1:%04lu ", ADC1);
    sprintf(buffer + strlen(buffer), "2:%04lu ", ADC2);
    sprintf(buffer + strlen(buffer), "3:%04lu", ADC3);
    u8g_DrawStr(&u8g,  0, 56, buffer);
    sprintf(buffer, "B");
    sprintf(buffer + strlen(buffer), "4:%04lu ", ADC4);
    sprintf(buffer + strlen(buffer), "5:%04lu ", ADC5);
    sprintf(buffer + strlen(buffer), "6:%04lu ", ADC6);
    sprintf(buffer + strlen(buffer), "7:%04lu", ADC7);
    buffer[32] = 0x0;

    u8g_DrawStr(&u8g,  0, 64, buffer);
}

const listItem_t mainMenuList[7] = {
    {"Spindle motor", &motortest, (uint8_t*)1},
    {"Left wheel", &motortest, (uint8_t*)2},
    {"Right wheel", &motortest, (uint8_t*)3},
    {"Sensor front", NULL, NULL},
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

currentDisp_t currentDisplay = {&M0, NULL, NULL, 0};

uint8_t lcdCounter = 0;
bool keypressed = false; // move to keypad and implement repeat key?

void menuListfnc(void *ptr)
{
    const listItem_t *thisItem = (listItem_t *) ptr;
    u8g_SetFontRefHeightExtendedText(&u8g);
    uint8_t h = u8g_GetFontAscent(&u8g) - u8g_GetFontDescent(&u8g);
    uint8_t w = u8g_GetWidth(&u8g);

    uint8_t count = 0;
    uint8_t *curItem = &currentDisplay.value;

    if (keypadGetKey() == KEYDOWN && !keypressed) {
        if ((thisItem + *curItem + 1)->itemName != NULL)(*curItem)++;
        keypressed = true;
    }
    if (keypadGetKey() == KEYUP && !keypressed) {
        if (*curItem != 0)(*curItem)--;
        keypressed = true;
    }
    if (keypadGetKey() == KEYOK && !keypressed) {
        if ((thisItem + *curItem)->command != NULL) {
            currentDisplay.command = (thisItem + *curItem)->command;
        }
        if ((thisItem + *curItem)->parm != NULL) {
            currentDisplay.parm = (thisItem + *curItem)->parm;
        }
        keypressed = true;
    }

    //Hold down for repeated press  || (keypressed && keypadGetTime() % 2)

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

void testfunc(void)
{
    // useful function! :P
#ifdef debugSemohosting
    printf("func! Parm: %u\r\n", (int)currentDisplay.parm); // check if null
#endif
    currentDisplay.command = NULL;
    currentDisplay.parm = NULL;
}

int pwmtest, a1, a2, a3, a4, a5, a6, a7, a8, a9 = 0;

void motortest(void)
{
    // useful function! :P

    if (keypadGetKey() == KEYSTART && !keypressed) {
        keypressed = true;

        // Configure PWM gpio2.0-2 as pwm1.1-3 Works but hangs (still doing pwm)
        //LPC_PINCON->PINSEL3 |= (1 << 9);      //LED Backlight  **TEST** PWM1.2


        LPC_PINCON->PINSEL4 |= (1 << 0);      //PWM1.1
        LPC_PINCON->PINSEL4 |= (1 << 2);      //PWM1.2
        LPC_PINCON->PINSEL4 |= (1 << 4);      //PWM1.3
        LPC_PINCON->PINMODE4 |= (3 << 4);
        //LPC_PINCON->PINMODE_OD2 |= (1 << 2);



        //http://www.ocfreaks.com/lpc2148-pwm-programming-tutorial/

        LPC_PWM1->PCR = 0;
        LPC_PWM1->PR = 12; // The TC is incremented every PR+1 cycles of PCLK.

        LPC_PWM1->MR0 = 1000; // 2khz
        LPC_PWM1->MR1 = 0; // PWM1
        LPC_PWM1->MR2 = 0; // PWM2
        LPC_PWM1->MR3 = 0; // PWM3
        LPC_PWM1->MCR = BIT(1);             // Reset on PWMMR0
        LPC_PWM1->LER = BIT(0) | BIT(1) | BIT(2) | BIT(3); // MR0 - MR3 enabled.

        LPC_PWM1->PCR = BIT(9) | BIT(10) | BIT(11); // PWMENA1 | PWMENA2 | PWMENA3
        LPC_PWM1->TCR = BIT(1);             // Counter Reset

        LPC_PWM1->TCR = BIT(0) | BIT(3);    //Counter Enable | PWM Enable

        //Spindle
        LPC_GPIO2->FIODIR |= PIN(13);
        LPC_GPIO3->FIODIR |= PIN(25);
        LPC_GPIO3->FIODIR |= PIN(26);

        //Right
        LPC_GPIO2->FIODIR |= PIN(9);
        LPC_GPIO2->FIODIR |= PIN(8);
        LPC_GPIO0->FIODIR |= PIN(0);

        // Right
        LPC_GPIO2->FIODIR |= PIN(4);
        LPC_GPIO2->FIODIR |= PIN(5);
        LPC_GPIO2->FIODIR |= PIN(6);
    }

    int enb, brk, dir = 0;
    int *enbPort, *brkPort, *dirPort, *pwmPort;

    if ((int)currentDisplay.parm == 1) {
        //Spindle
        enbPort = (int*) &LPC_GPIO2->FIOPIN;
        enb = PIN(13);
        brkPort = (int*) &LPC_GPIO3->FIOPIN;
        brk = PIN(25);
        dirPort = (int*) &LPC_GPIO3->FIOPIN;
        dir = PIN(26);
        pwmPort = (int*) &LPC_PWM1->MR3;
    }
    if ((int)currentDisplay.parm == 2) {
        //Left
        enbPort = (int*) &LPC_GPIO2->FIOPIN;
        enb = PIN(9);
        brkPort = (int*) &LPC_GPIO2->FIOPIN;
        brk = PIN(8);
        dirPort = (int*) &LPC_GPIO0->FIOPIN;
        dir = PIN(0);
        pwmPort = (int*) &LPC_PWM1->MR2;
    }
    if ((int)currentDisplay.parm == 3) {
        //Right
        enbPort = (int*) &LPC_GPIO2->FIOPIN;
        enb = PIN(4);
        brkPort = (int*) &LPC_GPIO2->FIOPIN;
        brk = PIN(5);
        dirPort = (int*) &LPC_GPIO2->FIOPIN;
        dir = PIN(6);
        pwmPort = (int*) &LPC_PWM1->MR1;
    }


    if (keypadGetKey() == KEY0 && !keypressed) {
        pwmtest = 0;
        keypressed = true;
        *pwmPort = pwmtest;
        LPC_PWM1->LER = BIT(0) | BIT(1) | BIT(2) | BIT(3); // MR0 - MR3 enabled.
    }

    if (keypadGetKey() == KEYUP && !keypressed) {
        pwmtest = pwmtest + 50;
        if (pwmtest > 1000) pwmtest = 1000;
        keypressed = true;
        *pwmPort = pwmtest;
        LPC_PWM1->LER = BIT(0) | BIT(1) | BIT(2) | BIT(3); // MR0 - MR3 enabled.
    }
    if (keypadGetKey() == KEYDOWN && !keypressed) {
        pwmtest = pwmtest - 50;
        if (pwmtest < 0) pwmtest = 0;
        keypressed = true;
        *pwmPort = pwmtest;
        LPC_PWM1->LER = BIT(0) | BIT(1) | BIT(2) | BIT(3); // MR0 - MR3 enabled.
    }

    if (keypadGetKey() == KEY1 && !keypressed) {
        keypressed = true;
        if (!a1) {
            a1 = 1;
            *enbPort |= enb;
        } else {
            a1 = 0;
            *enbPort &= ~enb;
        }

    }
    if (keypadGetKey() == KEY2 && !keypressed) {
        keypressed = true;
        if (!a2) {
            a2 = 1;
            *brkPort |= brk;
        } else {
            a2 = 0;
            *brkPort &= ~brk;
        }

    }
    if (keypadGetKey() == KEY3 && !keypressed) {
        keypressed = true;
        if (!a3) {
            a3 = 1;
            *dirPort |= dir;
        } else {
            *dirPort &= ~dir;
            a3 = 0;
        }
    }

    if (keypadGetKey() == KEY4 && !keypressed) {
        keypressed = true;
        if (!a4) {
            a4 = 1;
        } else {
            a4 = 0;
        }
    }
    if (keypadGetKey() == KEY5 && !keypressed) {
        keypressed = true;
        if (!a5) {
            a5 = 1;
        } else {
            a5 = 0;
        }
    }
    if (keypadGetKey() == KEY6 && !keypressed) {
        keypressed = true;
        if (!a6) {
            a6 = 1;
        } else {
            a6 = 0;
        }
    }

    if (keypadGetKey() == KEY7 && !keypressed) {
        keypressed = true;
        if (!a7) {
            a7 = 1;
        } else {
            a7 = 0;
        }
    }
    if (keypadGetKey() == KEY8 && !keypressed) {
        keypressed = true;
        if (!a7) {
            a8 = 1;
        } else {
            a8 = 0;
        }
    }
    if (keypadGetKey() == KEY9 && !keypressed) {
        keypressed = true;
        if (!a9) {
            a9 = 1;
        } else {
            a9 = 0;
        }
    }


    char tmp[1] = "";

    u8g_SetFont(&u8g, u8g_font_4x6);
    sprintf(buffer, "pwm:%u a1:%u a1:%u a3:%u a4:%u", pwmtest, a1, a2, a3, a4);
    u8g_DrawStr(&u8g,  0, (8 * 2), buffer);

    tmp[1] = 0x0;
    tmp[0] = ((LPC_GPIO0->FIOPIN >> 10) & 1) ? 0x31 : 0x30;
    sprintf(buffer, "p0.10: %s", tmp);
    tmp[0] = ((LPC_GPIO0->FIOPIN >> 27) & 1) ? 0x31 : 0x30;
    sprintf(buffer + strlen(buffer), " - p0.27: %s", tmp);
    u8g_DrawStr(&u8g,  0, (8 * 3), buffer);

    tmp[0] = ((LPC_GPIO0->FIOPIN >> 7) & 1) ? 0x31 : 0x30;
    sprintf(buffer, "p0.7: %s", tmp);
    tmp[0] = ((LPC_GPIO0->FIOPIN >> 8) & 1) ? 0x31 : 0x30;
    sprintf(buffer + strlen(buffer), " - p0.8: %s", tmp);
    tmp[0] = ((LPC_GPIO0->FIOPIN >> 9) & 1) ? 0x31 : 0x30;
    sprintf(buffer + strlen(buffer), " - p0.9 %s", tmp);
    u8g_DrawStr(&u8g,  0, (8 * 4), buffer);

    tmp[0] = (a1 ? 0x31 : 0x30);
    sprintf(buffer, "Brake(1): %s", tmp);
    tmp[0] = (a2 ? 0x31 : 0x30);
    sprintf(buffer, "Enable(2): %s", tmp);
    tmp[0] = (a3 ? 0x31 : 0x30);
    sprintf(buffer + strlen(buffer), " - F/R(3): %s", tmp);
    u8g_DrawStr(&u8g,  0, (8 * 5), buffer);

    sprintf(buffer, "0:%04lu ", ADC0);
    sprintf(buffer + strlen(buffer), "1:%04lu ", ADC1);
    sprintf(buffer + strlen(buffer), "2:%04lu ", ADC2);
    sprintf(buffer + strlen(buffer), "3:%04lu", ADC3);
    u8g_DrawStr(&u8g,  0, (8 * 7), buffer);

    sprintf(buffer, "4:%04lu ", ADC4);
    sprintf(buffer + strlen(buffer), "5:%04lu ", ADC5);
    sprintf(buffer + strlen(buffer), "6:%04lu ", ADC6);
    sprintf(buffer + strlen(buffer), "7:%04lu", ADC7);
    u8g_DrawStr(&u8g,  0, (8 * 8), buffer);

}

void menuBootfnc(void)
{
    lcdCounter++;
    if (lcdCounter > 100) {
        currentDisplay.selected = &M1;
        currentDisplay.command = NULL;
        currentDisplay.parm = NULL;
        lcdCounter = 0;
    }
}

void menuBootWarnfnc(void)
{
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
    uint8_t h = u8g_GetFontAscent(&u8g) - u8g_GetFontDescent(&u8g);

    u8g_DrawStr(&u8g,  0, (20 + h * 0), line1);
    u8g_DrawStr(&u8g,  0, (24 + h * 1), line2);
    u8g_DrawStr(&u8g,  0, (24 + h * 2), line3);
    u8g_DrawStr(&u8g,  0, (24 + h * 3), line4);
    u8g_DrawStr(&u8g,  0, (24 + h * 4), line5);
    u8g_DrawStr(&u8g,  0, (24 + h * 5), line6);
    u8g_DrawStr(&u8g,  0, (28 + h * 6), line7);
    if (keypadGetKey() == KEYOK && keypadGetTime() > 6) {
        currentDisplay.selected = &M2; // TODO next menu
        currentDisplay.command = NULL;
        currentDisplay.parm = NULL;
        keypressed = true;
    }

}

void lcdUpdate(void)
{
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
        uint8_t w = u8g_GetStrWidth(&u8g, curMenu->itemName);
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


        if (keypadGetKey() == KEYBACK && !keypressed) {
            //TODO: prev scrn
            currentDisplay.parm = NULL;
            currentDisplay.command = NULL;
            keypressed = true;
        }
        if (!keypadGetState()) {
            keypressed = false;
            // TODO: resetKeyTime fnc keypad.c
        }


    } while (u8g_NextPage(&u8g));
}