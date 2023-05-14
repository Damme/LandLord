
#ifndef ROSComms_H
#define ROSComms_H

#include "FreeRTOS.h"
#include "queue.h"
#include "common.h"

void ROSCommsRx_Task(void *pvParameters);
void ROSCommsTx_Task(void *pvParameters);
void SPI0TxQueue_Task(void *pvParameters);
void ROSCommsTest_Task(void *pvParameters);

void debug( const char* format, ... );

__attribute__((used)) int _write(int fd, char *ptr, int len);


#endif // ROSComms_H
