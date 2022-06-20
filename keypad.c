#include <stdio.h>

#include "keypad.h"
#include "common.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "queue.h"
#include "powermgmt.h"
//#include "motorctrl.h"
#include "global.h"

const uint8_t tblKey[4][4] = {
    { KEY8,     KEY9,  KEY0,    KEYSTART },
    { KEY6,     KEYOK, KEYDOWN, KEY7 },
    { KEYBACK,  KEYUP, KEY4,    KEY5 },
    { KEYHOME,  KEY1,  KEY2,    KEY3 },
};

uint8_t keyState, keyLastState, keyPessTime = 0;
uint8_t keypadRow, keypadCol, keypadPressedKey, keypadIsRow = 0;

void keypad_SetRow(bool row) {
    if (row) {
        keypadIsRow = 1;
        GPIO_FNC_PULL(KEYPAD_COL0, PINMODE_PULLDOWN);
        GPIO_FNC_PULL(KEYPAD_COL1, PINMODE_PULLDOWN);
        GPIO_FNC_PULL(KEYPAD_COL2, PINMODE_PULLDOWN);
        GPIO_FNC_PULL(KEYPAD_COL3, PINMODE_PULLDOWN);
        GPIO_FNC_PULL(KEYPAD_ROW0, PINMODE_PULLUP);
        GPIO_FNC_PULL(KEYPAD_ROW1, PINMODE_PULLUP);
        GPIO_FNC_PULL(KEYPAD_ROW2, PINMODE_PULLUP);
        GPIO_FNC_PULL(KEYPAD_ROW3, PINMODE_PULLUP);

    } else {
        keypadIsRow = 0;
        GPIO_FNC_PULL(KEYPAD_COL0, PINMODE_PULLUP);
        GPIO_FNC_PULL(KEYPAD_COL1, PINMODE_PULLUP);
        GPIO_FNC_PULL(KEYPAD_COL2, PINMODE_PULLUP);
        GPIO_FNC_PULL(KEYPAD_COL3, PINMODE_PULLUP);
        GPIO_FNC_PULL(KEYPAD_ROW0, PINMODE_PULLDOWN);
        GPIO_FNC_PULL(KEYPAD_ROW1, PINMODE_PULLDOWN);
        GPIO_FNC_PULL(KEYPAD_ROW2, PINMODE_PULLDOWN);
        GPIO_FNC_PULL(KEYPAD_ROW3, PINMODE_PULLDOWN);
    }
    vTaskDelay(xDelay25);
}


bool keypad_IsPressed() {
    uint8_t ret = 0;
    if (GPIO_CHK_PIN(KEYPAD_STOP1)) { // Stop
        ret = 1;
        goto ret;

    }
    if (GPIO_CHK_PIN(KEYPAD_POWER)) { // Power
        ret = 1;
        goto ret;

    }
    if (!keypadIsRow) {
        ret = ((GPIO_CHK_PIN(KEYPAD_COL0) | GPIO_CHK_PIN(KEYPAD_COL1) |
                GPIO_CHK_PIN(KEYPAD_COL2) | GPIO_CHK_PIN(KEYPAD_COL3)) > 0);
        goto ret;
    }
ret:
    return ret;
}

uint8_t keypad_ProcessKey() {
    uint8_t ret = 0;
    if (GPIO_CHK_PIN(KEYPAD_STOP1)) { // stop
        ret = KEYSTOP;
        goto ret;

    } else if (GPIO_CHK_PIN(KEYPAD_POWER)) { //Power
        ret = KEYPWR;
        goto ret;

    } else if (GPIO_CHK_PIN(KEYPAD_COL0))
        keypadCol = 0;
    else if (GPIO_CHK_PIN(KEYPAD_COL1))
        keypadCol = 1;
    else if (GPIO_CHK_PIN(KEYPAD_COL2))
        keypadCol = 2;
    else if (GPIO_CHK_PIN(KEYPAD_COL3))
        keypadCol = 3;
    keypad_SetRow(1);
    if (GPIO_CHK_PIN(KEYPAD_ROW0))
        keypadRow = 0;
    else if (GPIO_CHK_PIN(KEYPAD_ROW1))
        keypadRow = 1;
    else if (GPIO_CHK_PIN(KEYPAD_ROW2))
        keypadRow = 2;
    else if (GPIO_CHK_PIN(KEYPAD_ROW3))
        keypadRow = 3;
    keypad_SetRow(0);
    ret = tblKey[keypadCol][keypadRow];
ret:
    return ret;
}

uint8_t keypad_ProcessTask() {
    if (keypad_IsPressed()) {
        if (keyLastState == 0) {
            keypadPressedKey = keypad_ProcessKey();
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

uint8_t keypad_GetState() {
    return keyState;
}

uint8_t keypad_GetKey() {
    if (keyState > 0) {
        return keypadPressedKey;
    }
    return KEYNULL;
}

uint8_t keypad_GetTime() {
    return keyPessTime;
}

void keypad_Task(void *pvParameters) {
    // Init keypad
    GPIO_DIR_IN(KEYPAD_COL0);
    GPIO_DIR_IN(KEYPAD_COL1);
    GPIO_DIR_IN(KEYPAD_COL2);
    GPIO_DIR_IN(KEYPAD_COL3);
    GPIO_DIR_IN(KEYPAD_ROW0);
    GPIO_DIR_IN(KEYPAD_ROW1);
    GPIO_DIR_IN(KEYPAD_ROW2);
    GPIO_DIR_IN(KEYPAD_ROW3);

    GPIO_FNC_INV(KEYPAD_COL0, PINMODE_INV);
    GPIO_FNC_INV(KEYPAD_COL1, PINMODE_INV);
    GPIO_FNC_INV(KEYPAD_COL2, PINMODE_INV);
    GPIO_FNC_INV(KEYPAD_COL3, PINMODE_INV);
    GPIO_FNC_INV(KEYPAD_ROW0, PINMODE_INV);
    GPIO_FNC_INV(KEYPAD_ROW1, PINMODE_INV);
    GPIO_FNC_INV(KEYPAD_ROW2, PINMODE_INV);
    GPIO_FNC_INV(KEYPAD_ROW3, PINMODE_INV);

    GPIO_FNC_INV(KEYPAD_POWER, PINMODE_INV);

    keypad_SetRow(0);


    for (;;) {

        vTaskDelay(xDelay100);
        keypad_ProcessTask();

        if (keypad_GetKey() == KEYSTOP) {
            xMotorMsgType MotorMsg;
            MotorMsg.action = EMGSTOP;
            xQueueSendFromISR(xMotorMsgQueue, &MotorMsg, NULL);
        }
        /*if (keypad_GetKey() == KEYPWR && keypad_GetTime() > 6) {
            // TODO: Shut down motor! break etc, draws current from battery even in cpu-off!
            // TODO: display shutdown counter! (doesn't shutdown until released (hardware feature))
            // CPU in sleep??
            xPowerMgmtMsg PowerMsg;
            PowerMsg.xType = COMMAND_SHUTDOWN;
            PowerMsg.shutdown.xDelay = xDelay100;
            xQueueSend(xPowerMgmtMsgQueue, &PowerMsg, (TickType_t)0);
        }*/
#if LOWSTACKWARNING
        int stack = uxTaskGetStackHighWaterMark(NULL);
        if (stack < 50) printf("Task task_Keypad has %u words left in stack.\r\n", stack);
#endif

    }
}
