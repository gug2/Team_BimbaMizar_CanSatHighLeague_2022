/*
 * LSM6DSLTR.c
 *
 *  Created on: May 14, 2022
 *      Author: andre
 */
#include "LSM6DSLTR.h"

static SPI_HandleTypeDef* hspi;
static GPIO_TypeDef* CS_GPIO_PORT;
static uint16_t CS_GPIO_PIN;

static uint8_t address_data[2];
static uint8_t buf[14];

/* Чтение ID lsm6 - начало */
//HAL_StatusTypeDef spiStatus, spiStatus2;
/* Чтение ID lsm6 - конец*/

uint8_t LSM6DSLTR_ReadID() {
	uint8_t deviceID = LSM6DSLTR_ReadRegister(0x0F);

	/* Чтение ID lsm6 - начало */
	/*uint8_t reg = 0x0F| 0x80;
	LSM6DSLTR_EnableCSB();
	spiStatus = HAL_SPI_Transmit(hspi, &reg, 1, 1000);
	spiStatus2 = HAL_SPI_Receive(hspi, &deviceID, 1, 1000);
	LSM6DSLTR_DisableCSB();*/
	// ID чипа - 0x6A
	/* Чтение ID lsm6 - конец*/

	return deviceID;
}

LSM6DSLTR_Status LSM6DSLTR_Init(SPI_HandleTypeDef* hspix, GPIO_TypeDef* gpioPort, uint16_t gpioPin) {
	hspi = hspix;
	CS_GPIO_PORT = gpioPort;
	CS_GPIO_PIN = gpioPin;

	// reset the device
	////LSM6DSLTR_SoftReset();

	// disable I2C interface
	////LSM6DSLTR_I2cDisable();

	// delay for correct device start
	HAL_Delay(50);

	if(LSM6DSLTR_ReadID() != 0x6A) return LSM6DSLTR_DEVICE_ERROR;

	return LSM6DSLTR_DEVICE_OK;
}

void LSM6DSLTR_SetAccelerometerODR_FS(LSM6DSLTR_ODR odr, LSM6DSLTR_Accelerometer_Fullscale fs) {
	LSM6DSLTR_WriteTo(0x10, 0x00 | (odr << 4) | (fs << 2));
}

void LSM6DSLTR_SetGyroscopeODR_FS(LSM6DSLTR_ODR odr, LSM6DSLTR_Gyroscope_Fullscale fs, LSM6DSLTR_Settings_FS_125DPS flag) {
	if(odr > LSM6DSLTR_ODR_6660HZ) return;

	LSM6DSLTR_WriteTo(0x11, 0x00 | (odr << 4) | (fs << 2) | (flag << 1));
}

/* Soft device reset */
void LSM6DSLTR_SoftReset() {
	// read register current data
	uint8_t ctrl3_cData = LSM6DSLTR_ReadRegister(0x12);

	// 1 - go to soft reset, after that, bit is automatically cleared
	LSM6DSLTR_WriteTo(0x12, ctrl3_cData | (1 << 0));
}

void LSM6DSLTR_I2cDisable() {
	// read register current data
	uint8_t ctrl4_cData = LSM6DSLTR_ReadRegister(0x13);

	LSM6DSLTR_WriteTo(0x13, ctrl4_cData | (1 << 2));
}

// not used
void LSM6DSLTR_SoftMagnetometerCorrection() {
	uint8_t ctrl9_xlData = LSM6DSLTR_ReadRegister(0x18);

	LSM6DSLTR_WriteTo(0x18, ctrl9_xlData | (1 << 2));
}

// not used
uint8_t LSM6DSLTR_HasNewData(LSM6DSLTR_HAS_NEW_DATA_SELECTOR selector) {
	uint8_t status_regData = LSM6DSLTR_ReadRegister(0x1E);

	// check bit 0 - accelerometer
	// check bit 1 - gyroscope
	// check bit 2 - thermometer
	uint8_t mask = 0x00 | (1 << selector);

	if(status_regData & mask) {
		// new data available
		return 1;
	}

	return 0;
}

void LSM6DSLTR_ReadData(LSM6DSLTR_Data_Typedef* data) {
	uint8_t dataReg = 0x20 | 0x80;
	LSM6DSLTR_EnableCSB();
	HAL_SPI_Transmit(hspi, &dataReg, 1, LSM6DSLTR_SPI_TIMEOUT_MS);
	HAL_SPI_Receive(hspi, buf, 14, LSM6DSLTR_SPI_TIMEOUT_MS);
	LSM6DSLTR_DisableCSB();

	data->temperature = (buf[1] << 8) | buf[0];

	data->gyroscope.x = (buf[3] << 8) | buf[2];
	data->gyroscope.y = (buf[5] << 8) | buf[4];
	data->gyroscope.z = (buf[7] << 8) | buf[6];

	data->accelerometer.x = (buf[9] << 8) | buf[8];
	data->accelerometer.y = (buf[11] << 8) | buf[10];
	data->accelerometer.z = (buf[13] << 8) | buf[12];
}

uint8_t LSM6DSLTR_ReadRegister(uint8_t address) {
	uint8_t registerData = 0x00;

	// RW bit - 1
	address |= 0x80;

	LSM6DSLTR_EnableCSB();
	HAL_SPI_Transmit(hspi, &address, 1, LSM6DSLTR_SPI_TIMEOUT_MS);
	HAL_SPI_Receive(hspi, &registerData, 1, LSM6DSLTR_SPI_TIMEOUT_MS);
	LSM6DSLTR_DisableCSB();

	return registerData;
}

void LSM6DSLTR_WriteTo(uint8_t address, uint8_t data) {
	address_data[0] = address;
	address_data[1] = data;

	// RW bit - 0
	LSM6DSLTR_EnableCSB();
	HAL_SPI_Transmit(hspi, address_data, 2, LSM6DSLTR_SPI_TIMEOUT_MS);
	LSM6DSLTR_DisableCSB();
}

void LSM6DSLTR_EnableCSB() {
	HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO_PIN, GPIO_PIN_RESET);
}

void LSM6DSLTR_DisableCSB() {
	HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO_PIN, GPIO_PIN_SET);
}
