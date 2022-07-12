/*
 * LSM303DTR.h
 *
 *  Created on: 8 мая 2022 г.
 *      Author: andre
 */

#ifndef INC_LSM303DTR_H_
#define INC_LSM303DTR_H_

#include "stm32f4xx_hal.h"

#define LSM303DTR_SPI_TIMEOUT_MS 1000

struct LSM303DTR_sensorVector {
	int16_t x, y, z;
};

typedef struct LSM303DTR_Data_Typedef {
	struct LSM303DTR_sensorVector accelerometer;
	struct LSM303DTR_sensorVector compass;
	int16_t temperature;
} LSM303DTR_Data_Typedef;

typedef enum LSM303DTR_Status {
	LSM303DTR_DEVICE_ERROR,
	LSM303DTR_DEVICE_OK
} LSM303DTR_Status;

typedef enum LSM303DTR_Odr {
	LSM303DTR_ODR_POWER_DOWN,
	LSM303DTR_ODR_3_125HZ,
	LSM303DTR_ODR__6_25HZ,
	LSM303DTR_ODR_12_5HZ,
	LSM303DTR_ODR_25HZ,
	LSM303DTR_ODR_50HZ,
	LSM303DTR_ODR_100HZ,
	LSM303DTR_ODR_200HZ,
	LSM303DTR_ODR_400HZ,
	LSM303DTR_ODR_800HZ,
	LSM303DTR_ODR_1600HZ
} LSM303DTR_Odr;

typedef enum LSM303DTR_Fullscale {
	LSM303DTR_FS_2G,
	LSM303DTR_FS_4G,
	LSM303DTR_FS_6G,
	LSM303DTR_FS_8G,
	LSM303DTR_FS_16G
} LSM303DTR_Fullscale;

typedef enum LSM303DTR_Compass_Fullscale {
	LSM303DTR_FS_2GAUSS,
	LSM303DTR_FS_4GAUSS,
	LSM303DTR_FS_8GAUSS,
	LSM303DTR_FS_12GAUSS
} LSM303DTR_Compass_Fullscale;

typedef enum LSM303DTR_Compass_Mode {
	LSM303DTR_CONTINUOUS_CONVERSION,
	LSM303DTR_SINGLE_CONVERSION,
	LSM303DTR_POWER_DOWN
} LSM303DTR_Compass_Mode;

uint8_t LSM303DTR_ReadID();
LSM303DTR_Status LSM303DTR_Init(SPI_HandleTypeDef*, GPIO_TypeDef*, uint16_t);
void LSM303DTR_EnableThermometer();
void LSM303DTR_DisableThermometer();
void LSM303DTR_SetAccelerometerODR(LSM303DTR_Odr);
void LSM303DTR_SetCompassODR(LSM303DTR_Odr);
void LSM303DTR_SetAccelerometerFS(LSM303DTR_Fullscale);
void LSM303DTR_SetCompassFS(LSM303DTR_Compass_Fullscale);
void LSM303DTR_SetCompassMode(LSM303DTR_Compass_Mode);

void LSM303DTR_ReadData(LSM303DTR_Data_Typedef*);

uint8_t LSM303DTR_ReadRegister(uint8_t);
void LSM303DTR_WriteTo(uint8_t, uint8_t);
void LSM303DTR_EnableCSB();
void LSM303DTR_DisableCSB();

#endif /* INC_LSM303DTR_H_ */
