/*
 * LSM303DTR.c
 *
 *  Created on: May 8, 2022 г.
 *      Author: andre
 */
#include "LSM303DTR.h"

static SPI_HandleTypeDef* hspi;
static GPIO_TypeDef* CS_GPIO_PORT;
static uint16_t CS_GPIO_PIN;

static uint8_t thermometerOnBit = 0;

static uint8_t address_data[2];
static uint8_t buf[6];

uint8_t ID_303, REG;

uint8_t LSM303DTR_ReadID() {
	uint8_t deviceID = LSM303DTR_ReadRegister(0x0F);

	/**uint8_t reg = 0x0F | 0x80;

	LSM303DTR_EnableCSB();
	HAL_SPI_Transmit(hspi, &reg, 1, LSM303DTR_SPI_TIMEOUT_MS);
	HAL_SPI_Receive(hspi, &deviceID, 1, LSM303DTR_SPI_TIMEOUT_MS);
	LSM303DTR_DisableCSB();**/

	return deviceID;
}

LSM303DTR_Status LSM303DTR_Init(SPI_HandleTypeDef* hspix, GPIO_TypeDef* gpioPort, uint16_t gpioPin) {
	hspi = hspix;
	CS_GPIO_PORT = gpioPort;
	CS_GPIO_PIN = gpioPin;

	// delay for correct device start
	HAL_Delay(50);

	ID_303 = LSM303DTR_ReadID();
	if(ID_303 != 0x49) return LSM303DTR_DEVICE_ERROR;

	return LSM303DTR_DEVICE_OK;
}

void LSM303DTR_EnableThermometer() {
	// don't overwrite if thermometer is on
	if(thermometerOnBit == 0) {
		thermometerOnBit = 1;
		LSM303DTR_WriteTo(0x24, 0x00 | (thermometerOnBit << 7));
	}
}

void LSM303DTR_DisableThermometer() {
	// don't overwrite if thermometer is off
	if(thermometerOnBit != 0) {
		thermometerOnBit = 0;
		LSM303DTR_WriteTo(0x24, 0x00 | (thermometerOnBit << 7));
	}
}

void LSM303DTR_SetAccelerometerODR(LSM303DTR_Odr odr) {
	LSM303DTR_WriteTo(0x20, 0b00000111 | (odr << 4));

	/**REG = LSM303DTR_ReadRegister(0x20);**/
}

void LSM303DTR_SetCompassODR(LSM303DTR_Odr odr) {
	if(odr == LSM303DTR_ODR_POWER_DOWN || odr > LSM303DTR_ODR_100HZ) return;

	LSM303DTR_WriteTo(0x24, 0x00 | (thermometerOnBit << 7) | ((odr-1) << 2));
}

void LSM303DTR_SetAccelerometerFS(LSM303DTR_Fullscale fs) {
	LSM303DTR_WriteTo(0x21, 0x00 | (fs << 3));
}

void LSM303DTR_SetCompassFS(LSM303DTR_Compass_Fullscale fs) {
	LSM303DTR_WriteTo(0x25, 0x00 | (fs << 5));
}

void LSM303DTR_SetCompassMode(LSM303DTR_Compass_Mode mode) {
	// high-pass filter mode - normal mode
	//uint8_t ahpm = 0x01 << 6;
	// data from internal filter sent to registers
	//uint8_t afds = 0x01 << 5;
	//LSM303DTR_WriteTo(0x26, 0x00 | ahpm | afds | mode);

	LSM303DTR_WriteTo(0x26, 0x00 | mode);
}

void LSM303DTR_ReadData(LSM303DTR_Data_Typedef* data) {
	// RW bit - 1 (read)
	uint8_t dataReg = 0x28 | 0x80;
	// MS bit - 1 (auto address inc)
	dataReg |= (1 << 6);

	LSM303DTR_EnableCSB();
	HAL_SPI_Transmit(hspi, &dataReg, 1, LSM303DTR_SPI_TIMEOUT_MS);
	HAL_SPI_Receive(hspi, buf, 6, LSM303DTR_SPI_TIMEOUT_MS);
	LSM303DTR_DisableCSB();
	data->accelerometer.x = (buf[1] << 8) | buf[0];
	data->accelerometer.y = (buf[3] << 8) | buf[2];
	data->accelerometer.z = (buf[5] << 8) | buf[4];

	// RW bit - 1 (read)
	dataReg = 0x08 | 0x80;
	// MS bit - 1 (auto address inc)
	dataReg |= (1 << 6);

	LSM303DTR_EnableCSB();
	HAL_SPI_Transmit(hspi, &dataReg, 1, LSM303DTR_SPI_TIMEOUT_MS);
	HAL_SPI_Receive(hspi, buf, 6, LSM303DTR_SPI_TIMEOUT_MS);
	LSM303DTR_DisableCSB();
	data->compass.x = (buf[1] << 8) | buf[0];
	data->compass.y = (buf[3] << 8) | buf[2];
	data->compass.z = (buf[5] << 8) | buf[4];

	if(thermometerOnBit > 0) {
		// RW bit - 1 (read)
		dataReg = 0x05 | 0x80;
		// MS bit - 1 (auto address inc)
		dataReg |= (1 << 6);

		LSM303DTR_EnableCSB();
		HAL_SPI_Transmit(hspi, &dataReg, 1, LSM303DTR_SPI_TIMEOUT_MS);
		HAL_SPI_Receive(hspi, buf, 2, LSM303DTR_SPI_TIMEOUT_MS);
		LSM303DTR_DisableCSB();
		data->temperature = (buf[1] << 8) | buf[0];
	}
}

uint8_t LSM303DTR_ReadRegister(uint8_t address) {
	uint8_t registerData = 0x00;

	// RW bit - 1
	address |= 0x80;
	// MS bit - auto address increment (0)
	//address |= (0 << 6);

	LSM303DTR_EnableCSB();
	HAL_SPI_Transmit(hspi, &address, 1, LSM303DTR_SPI_TIMEOUT_MS);
	HAL_SPI_Receive(hspi, &registerData, 1, LSM303DTR_SPI_TIMEOUT_MS);
	LSM303DTR_DisableCSB();

	return registerData;
}

void LSM303DTR_WriteTo(uint8_t address, uint8_t data) {
	address_data[0] = address;
	address_data[1] = data;

	LSM303DTR_EnableCSB();
	HAL_SPI_Transmit(hspi, address_data, 2, LSM303DTR_SPI_TIMEOUT_MS);
	LSM303DTR_DisableCSB();
}

void LSM303DTR_EnableCSB() {
	HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO_PIN, GPIO_PIN_RESET);
}

void LSM303DTR_DisableCSB() {
	HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO_PIN, GPIO_PIN_SET);
}
