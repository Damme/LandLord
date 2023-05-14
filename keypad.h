#ifndef KEYPAD_H
    #define KEYPAD_H

    #include <stdint.h>
    #include <stdbool.h>

    uint8_t keypad_ProcessTask(void);
    uint8_t keypad_GetState(void);
    uint8_t keypad_GetKey(void);
    uint8_t keypad_GetTime(void);

    void keypad_Task(void *pvParameters);

#endif
