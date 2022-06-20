
#ifndef ROSComms_H
#define ROSComms_H

#include "FreeRTOS.h"
#include "queue.h"
#include "common.h"


void ROSComms_Task(void *pvParameters);
__attribute__((used)) int _write(int fd, char *ptr, int len);
//__attribute__((used)) int fputc(int ch, FILE *f);

#endif // ROSComms_H
