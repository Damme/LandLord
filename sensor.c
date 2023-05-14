#include <stdio.h>
#include "common.h"
#include "sensor.h"
#include "powermgmt.h"
#include "common.h"
#include "timers.h"
#include "event_groups.h"
#include "global.h"
#include "i2c.h"

#include "ROSComms.h"

#include <math.h>

#define RAIN_ADC_VALUE  3500 // 3840 = dry a coupld of hours after rain

typedef struct { // Ugh, the i2c sensors uses LSB/MSB differently...
    uint8_t X1;
    uint8_t X2;
    uint8_t Y1;
    uint8_t Y2;
    uint8_t Z1;
    uint8_t Z2;
} GenericSensor;

int32_t temperature2(int raw_value) {
    float a = 5.8849e-05;
    float b = -0.4366;
    float c = 809.9;
    float temp = a * raw_value * raw_value + b * raw_value + c;
    return temp*10;
}

// Test to remove the code below using a table
int32_t temp_linear_formula(int32_t raw) {
    int32_t result = (-0.27 * (raw - 2800) + 55);
    return result;
}

const uint16_t tempCalTbl[110] = {
    0xCF3, 0xCD6, 0xCB9, 0xC9B, 0xC7C, 0xC5A, 0xC3B, 0xC1B, 0xBFB, 0xBDB,
    0xBB5, 0xB93, 0xB72, 0xB50, 0xB2E, 0xB01, 0xADE, 0xABA, 0xA97, 0xA73,
    0xA46, 0xA23, 0x9FF, 0x9DB, 0x9B7, 0x984, 0x960, 0x93B, 0x917, 0x8F3,
    0x8C1, 0x89E, 0x87A, 0x857, 0x834, 0x800, 0x7DD, 0x7BA, 0x798, 0x776,
    0x743, 0x722, 0x701, 0x6E0, 0x6C0, 0x68E, 0x66F, 0x651, 0x633, 0x616,
    0x5E4, 0x5C7, 0x5AB, 0x590, 0x575, 0x545, 0x52B, 0x511, 0x4F8, 0x4DF,
    0x4B3, 0x49B, 0x484, 0x46E, 0x457, 0x42E, 0x419, 0x404, 0x3F0, 0x3DC,
    0x3B6, 0x3A3, 0x391, 0x37F, 0x36D, 0x34A, 0x339, 0x329, 0x319, 0x30A,
    0x2EA, 0x2DC, 0x2CD, 0x2C0, 0x2B2, 0x295, 0x288, 0x27C, 0x270, 0x264,
    0x249, 0x23E, 0x234, 0x229, 0x21F, 0x207, 0x1FD, 0x1F3, 0x1EA, 0x1E1,
    0x1CC, 0x1C4, 0x1BB, 0x1B3, 0x1AB, 0x198, 0x191, 0x18A, 0x183, 0x17C,
};

int32_t convert_temp(uint16_t raw_temp) {
    /* table holds 110 entries from -10°C to 100°C in full °C */
    int32_t fullDegree; /* full degrees */
    int32_t tenthDegree = 0; /* 10th-degrees */
    for (fullDegree = 0; fullDegree < (sizeof(tempCalTbl) / sizeof(uint16_t)); fullDegree++) {
        if (tempCalTbl[fullDegree] < raw_temp) { /* just one above */
            if (fullDegree > 0) { /* valid range? */
                --fullDegree; /* one back */
                for (tenthDegree = 0; (tenthDegree < 10); tenthDegree++) { /* search along gradient */
                    if ((tempCalTbl[fullDegree] - (tempCalTbl[fullDegree] - tempCalTbl[fullDegree + 1]) * tenthDegree / 10) <= raw_temp)
                        break;
                }
            }
            goto out;
        }
        if (tempCalTbl[fullDegree] == raw_temp) {
            tenthDegree = 0;
            goto out;
        }
    }
out:
    return tenthDegree + 10 * (fullDegree - 10);
}

// DB275 uses interrupt to read front sensor, need work and more testing.
void EINT3_IRQHandler(void) {
/*#ifndef LPC177x_8x // DB275
    uint32_t timer = LPC_TIM2->TC; // Get µs counter
    NVIC_DisableIRQ(EINT3_IRQn);

    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    if (timer == 0) LPC_TIM2->TCR = 1; // start if 0

    xQueueSendFromISR(xDIGMsgQueue, &timer, &xHigherPriorityTaskWoken);

    // also need muxing p0.21 & p0.22 for distance and left/right -> outside int in task-sensor

    xEventGroupSetBitsFromISR(xSensorEventGroup, BIT_DIG_INT, &xHigherPriorityTaskWoken);

    // Important! If not cleared system freezes
    LPC_GPIOINT->IO0IntClr = PIN(7) | PIN(8) | PIN(9) | PIN(10);
    NVIC_ClearPendingIRQ(EINT3_IRQn);
    NVIC_EnableIRQ(EINT3_IRQn);

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
#endif*/
    return;
}

volatile uint8_t lastpulsel = 0;
volatile uint8_t lastpulser = 0;
volatile uint8_t lastpulseb = 0;

void TIMER2_IRQHandler(void) {
    globaltickms++;
    watchdogSPI++;
    
    if (lastpulsel != GPIO_CHK_PIN(MOTOR_LEFT_PULSE)) {
        lastpulsel = GPIO_CHK_PIN(MOTOR_LEFT_PULSE);
        pulsecounterl++;
    }
    if (lastpulser != GPIO_CHK_PIN(MOTOR_RIGHT_PULSE)) {
        lastpulser = GPIO_CHK_PIN(MOTOR_RIGHT_PULSE);
        pulsecounterr++;
    }
    if (lastpulseb != GPIO_CHK_PIN(MOTOR_BLADE_PULSE)) {
        lastpulseb = GPIO_CHK_PIN(MOTOR_BLADE_PULSE);
        pulsecounterb++;
    }
    LPC_TIM2->IR  |= (1 << 0); // Reset interrupt MR0
}

void sensor_Task(void *pvParameters) {
    sensor_Init();
    xSensorMsgType sensor;
    GenericSensor accel;
    GenericSensor gyro;
    
    memset(&sensor, 0x00, sizeof(xSensorMsgType));
    xQueueOverwrite(xSensorQueue, &sensor);

    // Setup timer for pulse counter isr.
    LPC_SC->PCONP |= PCONP_PCTIM2;
    LPC_TIM2->TCR = 0x02;
    LPC_TIM2->PR  = 0x00;
    LPC_TIM2->MR0 = 1000 * (SystemCoreClock / 1000000) - 1; // 1ms
    LPC_TIM2->IR  = 0xff;
    LPC_TIM2->MCR = 0x03; // Trigger INT on match + reset TC
    NVIC_SetPriority(TIMER2_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(TIMER2_IRQn);
    LPC_TIM2->TCR = 0x01;

#ifdef LPC177x_8x
    I2C1Init();
#endif

// 3-axis, 12-bit/8-bit digital accelerometer
    I2C1_Send_Addr(MMA8452Q, 0x2a, 0x15); // ACTIVE | LNOISE | DR 200hz
    I2C1_Send_Addr(MMA8452Q, 0x0e, 0x00); // Set range to +/- 2g
    
// Three-axis digital output gyroscope
    I2C1_Send_Addr(L3GD20, 0x20, 0x0f); // CTRL1 Enable all
    //I2C1_Send_Addr(L3GD20, 0x21, 0x00); // CTRL2
    //I2C1_Send_Addr(L3GD20, 0x22, 0x00); // CTRL3
    I2C1_Send_Addr(L3GD20, 0x23, 0x40); // CTRL4 BLE (MSB lo add) 2000dps
    //I2C1_Send_Addr(L3GD20, 0x24, 0x80); // CTRL5 Reboot memory content

    I2C1_Send_Addr(L3GD20, 0x39, 0x01); // Low_ODR
   
    
    

    vTaskDelay(xDelay250);

    for (;;) {
        vTaskDelay(xDelay10);
        if (xQueuePeek(xSensorQueue, &sensor, TicksPerMS*10) == pdTRUE) {

            // handle_ADCMuxing(); // Only for LPC1768! ADC4 is muxed between 4 measurements.
            sensor.stuck = GPIO_CHK_PIN(SENSOR_STUCK);
            sensor.stuck2 = GPIO_CHK_PIN(SENSOR_STUCK2);
            sensor.door = GPIO_CHK_PIN(SENSOR_DOOR); 
            sensor.lift = GPIO_CHK_PIN(SENSOR_LIFT); // Might be swapped with Collision
            sensor.collision = GPIO_CHK_PIN(SENSOR_COLLISION);  // Might be swapped with LIFT
            sensor.stop = GPIO_CHK_PIN(SENSOR_STOP);
            sensor.door2 = GPIO_CHK_PIN(SENSOR_DOOR2);
            
            sensor.batteryCellLow = GPIO_CHK_PIN(SENSOR_BATT_BS);
            sensor.batteryCellHigh= GPIO_CHK_PIN(SENSOR_BATT_BH);

            sensor.CurrentPWMLeft = LPC_PWM1->MR4;
            sensor.CurrentPWMRight = LPC_PWM1->MR5;
            sensor.CurrentPWMSpindle = LPC_PWM1->MR1;

            sensor.motorpulseleft = pulsecounterl;
            sensor.motorpulseright = pulsecounterr;
            sensor.motorpulseblade = pulsecounterb;

 // MMA8452Q
            I2C1_Recv_Addr_Buf(MMA8452Q, 0x01, 1, sizeof(accel), &accel);
            sensor.AccelX = ((accel.X2 << 8) + accel.X1) >> 4;
            sensor.AccelY = ((accel.Y2 << 8) + accel.Y1) >> 4;
            sensor.AccelZ = ((accel.Z2 << 8) + accel.Z1) >> 4;
            if (sensor.AccelX > 2047) sensor.AccelX -= 4096;
            if (sensor.AccelY > 2047) sensor.AccelY -= 4096;
            if (sensor.AccelZ > 2047) sensor.AccelZ -= 4096;

// L3GD20  
            // If the MSb of the SUB field is 1, the SUB (register address) will be automatically incremented to allow multiple data read/write.
            I2C1_Recv_Addr_Buf(L3GD20, 0x28 | (1 << 7), 1, sizeof(gyro), &gyro);
            sensor.GyroYaw = (gyro.X1 << 8) + gyro.X2;
            sensor.GyroPitch = (gyro.Y1 << 8) + gyro.Y2;
            sensor.GyroRoll = (gyro.Z1 << 8) + gyro.Z2;
            //if (sensor.GyroYaw > INT16_MAX) sensor.GyroYaw -= UINT16_MAX+1;
            //if (sensor.GyroPitch > INT16_MAX) sensor.GyroPitch -= UINT16_MAX+1;
            //if (sensor.GyroRoll > INT16_MAX) sensor.GyroRoll -= UINT16_MAX+1;

            sensor.GyroYaw = sensor.GyroYaw * GYRO_SENSITIVITY_2000DPS;
            sensor.GyroPitch = sensor.GyroPitch * GYRO_SENSITIVITY_2000DPS;
            sensor.GyroRoll = sensor.GyroRoll * GYRO_SENSITIVITY_2000DPS;

            while (!(LPC_ADC->GDR & (1<<31))); // Wait for ADC conv. Done
            sensor.batteryTemp = temp_linear_formula(ADC_DR_RESULT(ANALOG_BATT_TEMP));
            sensor.batteryVolt = ADC_DR_RESULT(ANALOG_BATT_VOLT) * 100000 / 13068; // Measured and calculated.
            sensor.batteryChargeCurrent = ((ADC_DR_RESULT(ANALOG_BATT_CHARGE_A) + 24) * 0.8) - 146; // Trial and error :)
            if (sensor.batteryChargeCurrent < 0) sensor.batteryChargeCurrent = 0;
            sensor.motorRCurrent = ADC_DR_RESULT(ANALOG_MOTOR_R_AMP);
            sensor.motorLCurrent = ADC_DR_RESULT(ANALOG_MOTOR_L_AMP);
            sensor.motorBRpm = ADC_DR_RESULT(ANALOG_MOTOR_S_AMP);
            sensor.rainAnalog = ADC_DR_RESULT(ANALOG_RAIN);
            // TODO Not using same temperature conversion! ~0c = 3750raw ~7c = 3370raw ~22c = 3090raw ~30c = 3000raw
            //sensor.boardTemp = temperature2(ADC_DR_RESULT(ANALOG_BOARD_TEMP));
            sensor.boardTemp = ADC_DR_RESULT(ANALOG_BOARD_TEMP);
            if ( sensor.rainAnalog < RAIN_ADC_VALUE ) sensor.rain = 1; else sensor.rain = 0;

            xQueueOverwrite(xSensorQueue, &sensor);
            
        }
    }
}

