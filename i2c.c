#include <stdbool.h>
#include "i2c.h"
#include "common.h"
#include "global.h"
#include "FreeRTOS.h"

void I2C1Init() {
    LPC_SC->PCONP |= PCONP_PCI2C1;              // Power up I2C

    GPIO_PIN_FNC(SENSORS_SDA);
    GPIO_PIN_FNC(SENSORS_SDL);

    LPC_I2C1->SCLL = 300; // 16 bit SCH Duty Cycle Register
    LPC_I2C1->SCLH = 300; // 16 bit SCH Duty Cycle Register
// 3000 = 10kHz
// 1000 = 30kHz
// 300 = 100khz
    LPC_I2C1->CONSET = (1 << 6); // I2C interface enable.
}

int I2C1_WaitForSI() {
    const TickType_t xMaxWait = pdMS_TO_TICKS(10); // 10 ms timeout
    TickType_t xStartTime = xTaskGetTickCount();

    while ((LPC_I2C1->CONSET & (1 << CON_SI)) == 0x00) {
        if ((xTaskGetTickCount() - xStartTime) > xMaxWait) {
            return -1; // Timeout
        }
    }
    return 0; // Success
}

void I2C1_Start(void) {
    LPC_I2C1->CONCLR = (1 << CON_SI); // Clear SI
    LPC_I2C1->CONSET = (1 << CON_STA); // Set START condition
    if (I2C1_WaitForSI() != 0) {
        // Handle error (e.g., restart or reset I2C)
    }
    LPC_I2C1->CONCLR = (1 << CON_STA); // Clear START condition
}

void I2C1_Stop(void) {
    LPC_I2C1->CONCLR = (1 << CON_SI); // Clear SI
    LPC_I2C1->CONSET = (1 << CON_STO); // Set STOP condition
    // No need to wait for SI after stop condition
}

void I2C1_Send(const uint8_t data) {
    LPC_I2C1->DAT = data & 0xff; // Send data
    LPC_I2C1->CONCLR = (1 << CON_SI); // Clear SI
    if (I2C1_WaitForSI() != 0) {
        // Handle error
    }
}

void I2C1_Send_Addr(const uint8_t addr, const uint8_t sub, const uint8_t data) {
    I2C1_Start();
    I2C1_Send(addr);
    I2C1_Send(sub);
    I2C1_Send(data);
    I2C1_Stop();
}

uint8_t I2C1_Recv(bool ack) {
    LPC_I2C1->CONCLR = (1 << CON_SI); // Clear SI
    if (ack) {
        LPC_I2C1->CONSET = (1 << CON_AA); // Acknowledge
    } else {
        LPC_I2C1->CONCLR = (1 << CON_AA); // Not acknowledge
    }
    if (I2C1_WaitForSI() != 0) {
        // Handle error
    }
    return (uint8_t)(LPC_I2C1->DAT & 0xff); // Read byte
}

void I2C1_Recv_Addr_Buf(const uint8_t addr, const uint8_t sub, bool ack, const uint8_t len, uint8_t *buf) {
    I2C1_Start();
    I2C1_Send(addr);
    I2C1_Send(sub);
    I2C1_Stop();
    vTaskDelay(pdMS_TO_TICKS(1)); // Adjust delay as needed
    I2C1_Start();
    I2C1_Send(addr + 1);
    for (int i = 0; i < len; i++) {
        if (i < (len - 1)) {
            buf[i] = I2C1_Recv(1); // Send ACK for all but the last byte
        } else {
            buf[i] = I2C1_Recv(0); // Do not send ACK for the last byte
        }
    }
    I2C1_Stop();
}

uint8_t I2C1_Recv_Addr(const uint8_t addr, const uint8_t sub) {
    uint8_t val = 0;
    I2C1_Recv_Addr_Buf(addr, sub, false, 1, &val);
    return val;
}