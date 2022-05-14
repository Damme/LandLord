#ifndef I2C_H
#define I2C_H

#include "FreeRTOS.h"

#define 	CON_AA		2
#define		CON_SI		3
#define		CON_STO		4
#define 	CON_STA		5
#define 	CON_I2EN	6

#define     MMA8452Q    0x38
#define     L3GD20      0xD4
#define     LSM303DLHC  0x32 // not in default configuration!

#define L3GD20_WHO_AM_I 0x0F
#define L3GD20_STATUS_REG 0x27
#define L3GD20_OUT_TEMP 0x26

#define MMA8452Q_WHO_AM_I 0x0D

void I2C1_Start (void);
void I2C1_Stop (void);

void I2C1_Send (const uint8_t data);
void I2C1_Send_Addr (const uint8_t addr, const uint8_t sub, const uint8_t data);

uint8_t I2C1_Recv (void);
uint8_t I2C1_Recv_Ack (void);
uint8_t I2C1_Recv_Addr (const uint8_t addr, const uint8_t sub, const bool ack);
void I2C1_Recv_Addr_Buf (const uint8_t addr, const uint8_t sub, const bool ack, const uint8_t len, uint8_t *buf);

#endif // I2C_H
