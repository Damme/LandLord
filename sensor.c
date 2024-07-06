#include <stdio.h>
#include "common.h"
#include "sensor.h"
#include "powermgmt.h"
#include "common.h"
#include "timers.h"
#include "event_groups.h"
#include "global.h"
#include "i2c.h"
#include "cJSON.h"

#include "ROSComms.h"

#include <math.h>

#define buflen  250

#define RAIN_ADC_VALUE  3500 // 3840 = dry a couple of hours after rain
#define ALPHA 25
#define SCALE 1000

#define GYRO_SENSITIVITY_DPS 0.00875
#define ACCEL_SENSITIVITY_LSB 0.00981
#define GRAVITY 9.80665


typedef struct { // Ugh, the i2c sensors uses LSB/MSB differently...
    uint8_t X1;
    uint8_t X2;
    uint8_t Y1;
    uint8_t Y2;
    uint8_t Z1;
    uint8_t Z2;
} GenericSensor;

int32_t imu_temp_offset_z(int32_t raw) {
    int32_t result = round(-0.565f * raw + 147.4f);
    return result;
}

int32_t conv_board_temp(int32_t raw) {
    int32_t result = round(-0.3483 * raw + 1315);
    return result;
}

// Test to remove the code below using a table
int32_t conv_batt_temp(int32_t raw) {
    int32_t result = round(-0.27 * (raw - 2800) + 55);
    return result;
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

volatile bool lastpulsel = 0;
volatile bool lastpulser = 0;
volatile bool lastpulseb = 0;

void TIMER2_IRQHandler(void) {
    sensorMsg.globalticksms++;
    sensorMsg.watchdogSPI++;
    
    if (lastpulsel != GPIO_CHK_PIN(MOTOR_LEFT_PULSE)) {
        lastpulsel = GPIO_CHK_PIN(MOTOR_LEFT_PULSE);
        sensorMsg.motorPulseLeft++;
    }
    if (lastpulser != GPIO_CHK_PIN(MOTOR_RIGHT_PULSE)) {
        lastpulser = GPIO_CHK_PIN(MOTOR_RIGHT_PULSE);
        sensorMsg.motorPulseRight++;
    }
    if (lastpulseb != GPIO_CHK_PIN(MOTOR_BLADE_PULSE)) {
        lastpulseb = GPIO_CHK_PIN(MOTOR_BLADE_PULSE);
        sensorMsg.motorPulseBlade++;
    }
    LPC_TIM2->IR  |= (1 << 0); // Reset interrupt MR0
}

void sensor_Task(void *pvParameters) {
    sensor_Init();
    char local_txbuf[buflen+1];
    GenericSensor accel = {0,0,0,0,0,0};
    GenericSensor gyro = {0,0,0,0,0,0};

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(40);
    
    uint32_t ema_batteryVolt = 25000;
    uint32_t ema_batteryChargeCurrent = 0;

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
    I2C1_Send_Addr(L3GD20, 0x20, 0x6F); // CTRL1 Enable all
    I2C1_Send_Addr(L3GD20, 0x21, 0x00); // CTRL2
    I2C1_Send_Addr(L3GD20, 0x22, 0x00); // CTRL3
    I2C1_Send_Addr(L3GD20, 0x23, 0x50); // CTRL4 BLE (MSB lo add) 
    I2C1_Send_Addr(L3GD20, 0x24, 0x10); // CTRL5 Reboot memory content
    vTaskDelay(xDelay100);

    //I2C1_Send_Addr(L3GD20, 0x39, 0x01); // Low_ODR
    int32_t rawYaw = 0;
    int32_t rawPitch = 0;
    int32_t rawRoll = 0;
    int32_t offsetRoll;

#define SAMPLES 2000

    for (int i=0;  i < SAMPLES; i++) {
        I2C1_Recv_Addr_Buf(L3GD20, 0x28 | (1 << 7), 1, sizeof(gyro), (uint8_t*)&gyro); // Bug? seem to freeze sensor task? i2c code!
        rawYaw += (int16_t)((gyro.X1 << 8) | gyro.X2);
        rawPitch += (int16_t)((gyro.Y1 << 8) | gyro.Y2);
        rawRoll += (int16_t)((gyro.Z1 << 8) | gyro.Z2);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    offsetRoll = imu_temp_offset_z(conv_board_temp(ADC_DR_RESULT(ANALOG_BOARD_TEMP)));
    rawYaw = round(rawYaw / SAMPLES);
    rawPitch = round(rawPitch / SAMPLES);
    rawRoll = round(rawRoll / SAMPLES) - offsetRoll;

   
    vTaskDelay(xDelay10);
    xLastWakeTime = xTaskGetTickCount();
  
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // handle_ADCMuxing(); // Only for LPC1768! ADC4 is muxed between 4 measurements.

        sensorMsg.stuck = GPIO_CHK_PIN(SENSOR_STUCK);
        sensorMsg.stuck2 = GPIO_CHK_PIN(SENSOR_STUCK2);
        sensorMsg.door = GPIO_CHK_PIN(SENSOR_DOOR); 
        sensorMsg.lift = GPIO_CHK_PIN(SENSOR_LIFT); // Might be swapped with Collision
        sensorMsg.collision = GPIO_CHK_PIN(SENSOR_COLLISION);  // Might be swapped with LIFT
        sensorMsg.stop = GPIO_CHK_PIN(SENSOR_STOP);
        sensorMsg.door2 = GPIO_CHK_PIN(SENSOR_DOOR2);
        
        sensorMsg.batteryCellLow = GPIO_CHK_PIN(SENSOR_BATT_BS);
        sensorMsg.batteryCellHigh= GPIO_CHK_PIN(SENSOR_BATT_BH);

        sensorMsg.currentPWMLeft = LPC_PWM1->MR4;
        sensorMsg.currentPWMRight = LPC_PWM1->MR5;
        sensorMsg.currentPWMBlade = LPC_PWM1->MR1;

// MMA8452Q
        I2C1_Recv_Addr_Buf(MMA8452Q, 0x01, 1, sizeof(accel), (uint8_t*)&accel);
        sensorMsg.accelX = ((accel.X2 << 8) + accel.X1) >> 4;
        sensorMsg.accelY = ((accel.Y2 << 8) + accel.Y1) >> 4;
        sensorMsg.accelZ = ((accel.Z2 << 8) + accel.Z1) >> 4;
        if (sensorMsg.accelX > 2047) sensorMsg.accelX -= 4096;
        if (sensorMsg.accelY > 2047) sensorMsg.accelY -= 4096;
        if (sensorMsg.accelZ > 2047) sensorMsg.accelZ -= 4096;

        sensorMsg.accelX *= GRAVITY;
        sensorMsg.accelY *= GRAVITY;
        sensorMsg.accelZ *= GRAVITY;

// L3GD20  
        I2C1_Recv_Addr_Buf(L3GD20, 0x28 | (1 << 7), 1, sizeof(gyro), (uint8_t*)&gyro);
        offsetRoll = imu_temp_offset_z(conv_board_temp(ADC_DR_RESULT(ANALOG_BOARD_TEMP)));

        sensorMsg.gyroYaw = ((int16_t)((gyro.X1 << 8) | gyro.X2) - rawYaw) * GYRO_SENSITIVITY_500DPS * (M_PI / 180.0);
        sensorMsg.gyroPitch = ((int16_t)((gyro.Y1 << 8) | gyro.Y2) - rawPitch) * GYRO_SENSITIVITY_500DPS * (M_PI / 180.0);
        sensorMsg.gyroRoll = (((int16_t)((gyro.Z1 << 8) | gyro.Z2) - rawRoll + offsetRoll) * GYRO_SENSITIVITY_500DPS * (M_PI / 180.0));

        // IMU Data:
        cJSON* root = cJSON_CreateObject();
        cJSON* obj = cJSON_CreateObject();
        cJSON_AddItemToObject(obj, "Yaw", cJSON_CreateNumber(round(sensorMsg.gyroYaw * 100000)));
        cJSON_AddItemToObject(obj, "Pitch", cJSON_CreateNumber(round(sensorMsg.gyroPitch * 100000)));
        cJSON_AddItemToObject(obj, "Roll", cJSON_CreateNumber(round(sensorMsg.gyroRoll * 100000)));
        cJSON_AddItemToObject(obj, "AccX", cJSON_CreateNumber(round(sensorMsg.accelX)));
        cJSON_AddItemToObject(obj, "AccY", cJSON_CreateNumber(round(sensorMsg.accelY)));
        cJSON_AddItemToObject(obj, "AccZ", cJSON_CreateNumber(round(sensorMsg.accelZ)));
        cJSON_AddItemToObject(root, "I2C_IMU", obj);
        cJSON_PrintPreallocated(root, local_txbuf, buflen, false);
        xQueueSend(RosTxQueue, local_txbuf, xDelay10);
        cJSON_Delete(root);

        // Is this really necessary? test to disable wait for adc done to test stability
        //while (!(LPC_ADC->GDR & (1<<31))); // Wait for ADC conv. Done
        sensorMsg.batteryTemp = conv_batt_temp(ADC_DR_RESULT(ANALOG_BATT_TEMP));

        ema_batteryVolt = ((ALPHA * (ADC_DR_RESULT(ANALOG_BATT_VOLT) * 100000 / 13068)) + ((SCALE - ALPHA) * ema_batteryVolt)) / SCALE;
        sensorMsg.batteryVolt = ema_batteryVolt;
        ema_batteryChargeCurrent = ((ALPHA * (((ADC_DR_RESULT(ANALOG_BATT_CHARGE_A) + 24) * 0.8) - 146)) + ((SCALE - ALPHA) * ema_batteryChargeCurrent)) / SCALE;
        sensorMsg.batteryChargeCurrent = ema_batteryChargeCurrent;

        if (sensorMsg.batteryChargeCurrent < 0) sensorMsg.batteryChargeCurrent = 0;
        sensorMsg.motorCurrentRight = ADC_DR_RESULT(ANALOG_MOTOR_R_AMP);
        sensorMsg.motorCurrentLeft = ADC_DR_RESULT(ANALOG_MOTOR_L_AMP);
        sensorMsg.motorCurrentBlade = ADC_DR_RESULT(ANALOG_MOTOR_S_AMP);
        sensorMsg.rainAnalog = ADC_DR_RESULT(ANALOG_RAIN);
        sensorMsg.boardTemp = conv_board_temp(ADC_DR_RESULT(ANALOG_BOARD_TEMP));
        if ( sensorMsg.rainAnalog < RAIN_ADC_VALUE ) sensorMsg.rain = 1; else sensorMsg.rain = 0;

    }
}