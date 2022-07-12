/*
 * LSM6DSLTR.c
 *
 *  Created on: June 9, 2022
 *      Author: andre
 */
#include "STLM75M2F.h"

static I2C_HandleTypeDef* hi2c;

uint8_t canSendRequest = 1;
uint8_t stlm_buf[2];
static int16_t stlm_temp = 0;

// After i2c transmit Callback
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2cx) {
	if(hi2cx == hi2c) {
		HAL_I2C_Master_Receive_IT(hi2c, STLM75M2F_I2C_ADDRESS << 1, stlm_buf, 2);
	}
}

// After i2c receive Callback
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2cx) {
	if(hi2cx == hi2c) {
		stlm_temp = (stlm_buf[0] << 8) | stlm_buf[1];

		// clear bit 15
		stlm_temp &= 0x7FFF;
	}
}

// max conversion time - 150ms
STLM75M2F_Status STLM75M2F_Init(I2C_HandleTypeDef* hi2cx) {
	hi2c = hi2cx;

	return STLM75M2F_DEVICE_OK;
}

int16_t STLM75M2F_GetTemperature() {
	//uint8_t stlm_buf[2];

	// choose temperature register - 0x00
	// choose configuration register - 0x01
	// choose hysteresis register - 0x02
	// choose overtemperature register - 0x03
	uint8_t reg = 0x00;
	////HAL_I2C_Master_Transmit(hi2c, STLM75M2F_I2C_ADDRESS << 1, &reg, 1, STLM75M2F_I2C_TIMEOUT_MS);
	////HAL_I2C_Master_Receive(hi2c, STLM75M2F_I2C_ADDRESS << 1, stlm_buf, 2, STLM75M2F_I2C_TIMEOUT_MS);
	if(canSendRequest) {
		HAL_I2C_Master_Transmit_IT(hi2c, STLM75M2F_I2C_ADDRESS << 1, &reg, 1);
	}

	//uint16_t rawTemp = (stlm_buf[0] << 8) | stlm_buf[1];

	// clear bit 15
	//rawTemp &= 0x7FFF;

	//return rawTemp;
	return stlm_temp;
}
