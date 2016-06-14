#include "keypad.h"

const uint8_t tblKey[4][4] = {
    { KEY8,     KEY9,  KEY0,    KEYSTART },
    { KEY6,     KEYOK, KEYDOWN, KEY7 },
    { KEYBACK,  KEYUP, KEY4,    KEY5 },
    { KEYHOME,  KEY1,  KEY2,    KEY3 },
};
uint8_t keyState, keyLastState = 0;
uint8_t keyPessTime = 0;

uint8_t keypadRow, keypadCol, keypadPressedKey;

void KeypadSetRow(bool row)
{
    if (row) {
        LPC_GPIO1->FIODIR |= (PIN(0) | PIN(1) | PIN(4) | PIN(8));
        LPC_GPIO1->FIODIR &= ~(PIN(9) | PIN(10) | PIN(14) | PIN(15));

        // TODO: needs cleanup

        LPC_PINCON->PINMODE2 &= ~(3 << 0);
        LPC_PINCON->PINMODE2 &= ~(3 << 2);
        LPC_PINCON->PINMODE2 &= ~(3 << 8);
        LPC_PINCON->PINMODE2 &= ~(3 << 16);

        LPC_PINCON->PINMODE2 |= (3 << 18);
        LPC_PINCON->PINMODE2 |= (3 << 20);
        LPC_PINCON->PINMODE2 |= (3 << 28);
        LPC_PINCON->PINMODE2 |= ((uint32_t)3 << 30);

        LPC_GPIO1->FIOPIN &= ~0xC600;
        LPC_GPIO1->FIOPIN |= 0x113;
    } else {
        LPC_GPIO1->FIODIR |= (1 << 9);
        LPC_GPIO1->FIODIR |= (1 << 10);
        LPC_GPIO1->FIODIR |= (1 << 14);
        LPC_GPIO1->FIODIR |= (1 << 15);

        LPC_GPIO1->FIODIR &= ~(1 << 0);
        LPC_GPIO1->FIODIR &= ~(1 << 1);
        LPC_GPIO1->FIODIR &= ~(1 << 4);
        LPC_GPIO1->FIODIR &= ~(1 << 8);

        LPC_PINCON->PINMODE2 &= ~(3 << 18);
        LPC_PINCON->PINMODE2 &= ~(3 << 20);
        LPC_PINCON->PINMODE2 &= ~(3 << 28);
        LPC_PINCON->PINMODE2 &= ~((uint32_t)3 << 30);

        LPC_PINCON->PINMODE2 |= (3 << 0);
        LPC_PINCON->PINMODE2 |= (3 << 2);
        LPC_PINCON->PINMODE2 |= (3 << 8);
        LPC_PINCON->PINMODE2 |= (3 << 16);

        LPC_GPIO1->FIOPIN &= ~0x113;
        LPC_GPIO1->FIOPIN |= 0xC600;
    }
}


bool keypadIsPressed()
{
    uint8_t ret = 0;
    if (LPC_GPIO1->FIOPIN & (1 << 17)) { // stop
        ret = 1;
        goto ret;
    }
    if (!(LPC_GPIO1->FIOPIN & (1 << 28))) { //Power
        ret = 1;
        goto ret;
    }
    if (!(LPC_GPIO1->FIODIR & 1)) {
        ret = ((LPC_GPIO1->FIOPIN & 0x113) > 0);
        goto ret;
    }
ret:
    return ret;
}

uint8_t keypadProcessKey()
{
    uint8_t ret = 0;
    if (LPC_GPIO1->FIOPIN & (1 << 17)) { // stop
        ret = KEYSTOP;
        goto ret;
    } else if (!(LPC_GPIO1->FIOPIN & (1 << 28))) { //Power
        ret = KEYPWR;
        goto ret;
    }
    if (LPC_GPIO1->FIOPIN & (1<<0))
        keypadRow = 0;
    else if (LPC_GPIO1->FIOPIN & (1<<1))
        keypadRow = 1;
    else if (LPC_GPIO1->FIOPIN & (1<<4))
        keypadRow = 2;
    else if (LPC_GPIO1->FIOPIN & (1<<8))
        keypadRow = 3;
    KeypadSetRow(1);
    vTaskDelay(xDelay25);
    if (LPC_GPIO1->FIOPIN & (1<<9))
        keypadCol = 0;
    else if (LPC_GPIO1->FIOPIN & (1<<10))
        keypadCol = 1;
    else if (LPC_GPIO1->FIOPIN & (1<<14))
        keypadCol = 2;
    else if (LPC_GPIO1->FIOPIN & (1<<15))
        keypadCol = 3;
    KeypadSetRow(0);
    ret = tblKey[keypadRow][keypadCol];
ret:
    return ret;
}

uint8_t keypadProcessTask()
{
    if (keypadIsPressed()) {
        if (keyLastState == 0) {
            keypadPressedKey = keypadProcessKey();
            keyState = 1;
            goto ret;
        }
        if (keyLastState == 1) {    // down
            keyState = 2;
            keyPessTime = 0;
            goto ret;
        }
        if (keyLastState == 2) {    // hold
            keyPessTime++;
            goto ret;
        }
    } else if (keyLastState == 1 || keyLastState == 2) {    // released
        keyState = 3;
        goto ret;
    }
    keyState = 0;  //normal
ret:
    keyLastState = keyState;
    return keyState;
}

uint8_t keypadGetState()
{
    return keyState;
}

uint8_t keypadGetKey()
{
    if (keyState > 0) {
        return keypadPressedKey;
    }
    return 255;
}

uint8_t keypadGetTime()
{
    return keyPessTime;
}
