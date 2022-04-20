#include <stdio.h>

#include "common.h"
#include "redirect.h"

__attribute__((used)) int _write(int fd, char *ptr, int len) {
    int i = 0;
    while (*ptr && (i < len)) {
        ITM_SendChar(*ptr);
        i++;
        ptr++;
    }
    return i;
}
