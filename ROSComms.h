
#ifndef ROSComms_H
#define ROSComms_H

#include "FreeRTOS.h"
#include "queue.h"
#include "common.h"


void ROSComms_Task(void *pvParameters);
void debug( const char* format, ... );

__attribute__((used)) int _write(int fd, char *ptr, int len);


#endif // ROSComms_H
