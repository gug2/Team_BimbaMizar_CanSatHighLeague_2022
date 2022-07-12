/*
 * LSM6DSLTR.h
 *
 *  Created on: June 9, 2022
 *      Author: andre
 */

#ifndef INC_STLM75M2F_H_
#define INC_STLM75M2F_H_

#include "stm32f4xx_hal.h"

#define STLM75M2F_I2C_ADDRESS 0b01001000
#define STLM75M2F_I2C_TIMEOUT_MS 1000

typedef enum STLM75M2F_Status{
	STLM75M2F_DEVICE_ERROR,
	STLM75M2F_DEVICE_OK
} STLM75M2F_Status;

STLM75M2F_Status STLM75M2F_Init(I2C_HandleTypeDef*);

int16_t STLM75M2F_GetTemperature();

#endif /* INC_STLM75M2F_H_ */
