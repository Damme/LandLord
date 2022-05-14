#include <stdbool.h>
#include "i2c.h"
#include "common.h"
#include "global.h"
#include "FreeRTOS.h"

void I2C1Init() {
    GPIO_PIN_FNC(SENSORS_SDA);
    GPIO_PIN_FNC(SENSORS_SDL);

    LPC_SC->PCONP |= PCONP_PCI2C1;              // Power up I2C

    LPC_I2C1->SCLL = 3000; // 16 bit SCH Duty Cycle Register
    LPC_I2C1->SCLH = 3000; // 16 bit SCH Duty Cycle Register
// 1000 = 30kHz
// 300 = 100khz
    LPC_I2C1->CONSET = (1<<6); // I2C interface enable. 
}

void I2C1_Start (void) {
    LPC_I2C1->CONCLR = (1<<CON_SI); //Clear SI
    LPC_I2C1->CONSET = (1<<CON_STA); //Set START condition
    while ( (LPC_I2C1->CONSET&(1<<CON_SI)) == 0x00); //Wait for complete
    LPC_I2C1->CONCLR = (1<<CON_STA); //Clear START condition
}
void I2C1_Stop (void) {
    if ((LPC_I2C1->CONSET&(1<<CON_STA)) != 0x00) LPC_I2C1->CONCLR=CON_STA; //Make sure start bit is not active 
    LPC_I2C1->CONCLR = (1<<CON_SI); //Clear SI
    LPC_I2C1->CONSET = (1<<CON_STO); //Set STOP condition
}

void I2C1_Send (const uint8_t data) {
    if ((LPC_I2C1->CONSET&(1<<CON_STA)) != 0x00) LPC_I2C1->CONCLR = (1<<CON_STA); //Make sure start bit is not active
    LPC_I2C1->DAT = data&0xff; //Send data
    LPC_I2C1->CONCLR = (1<<CON_SI); //Clear SI
    while ((LPC_I2C1->CONSET & (1<<CON_SI)) == 0x00); //Wait
}

void I2C1_Send_Addr (const uint8_t addr, const uint8_t sub, const uint8_t data) {
    I2C1_Start();
    I2C1_Send(addr);
    I2C1_Send(sub);
    I2C1_Send(data);
    I2C1_Stop();
}

uint8_t I2C1_Recv (void) {
	LPC_I2C1->CONCLR = (1<<CON_SI)|(1<<CON_AA); //Clear SI & AA
	while ((LPC_I2C1->CONSET & (1<<CON_SI)) == 0x00); //Wait
	return (uint8_t) (LPC_I2C1->DAT & 0xff); //Read byte
}

uint8_t I2C1_Recv_Ack (void) {
	LPC_I2C1->CONCLR = (1<<CON_SI); //Clear SI and set AA
	LPC_I2C1->CONSET = (1<<CON_AA);
	while ((LPC_I2C1->CONSET & (1<<CON_SI)) == 0x00); //Wait
	return (uint8_t) (LPC_I2C1->DAT & 0xff); //Read byte
}

uint8_t I2C1_Recv_Addr (const uint8_t addr, const uint8_t sub, const bool ack) {
    uint8_t val = 0;
    I2C1_Start();
    I2C1_Send(addr);
    I2C1_Send(sub);
    I2C1_Start();
    I2C1_Send(addr+1);
    if (ack) {
        val = I2C1_Recv_Ack();
    } else {
        val = I2C1_Recv();
    }
    //I2C1_Stop();
    return val;
}

void I2C1_Recv_Addr_Buf (const uint8_t addr, const uint8_t sub, bool ack, const uint8_t len, uint8_t *buf) {
    I2C1_Start();
    I2C1_Send(addr);
    I2C1_Send(sub);
    I2C1_Start();
    I2C1_Send(addr+1);
    for ( int i = 0 ; i < len ; i++ ) {
        //if (!(i < len)) ack = 0;
        if (ack) {
            buf[i] = I2C1_Recv_Ack();
        } else {
            buf[i] = I2C1_Recv();
        }
    }
    //I2C1_Stop();
    return;
}