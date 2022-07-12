/*
 * LSM303DTR.h
 *
 *  Created on: 16 июня 2022 г.
 *      Author: andre
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

#define GPS_BUFFER_SIZE 128

typedef struct GPS_Data_Typedef {
	float hhmmss_ms;
	char packetValid, NS, EW;
	uint32_t latitude1, latitude2;
	uint32_t longitude1, longitude2;
} GPS_Data_Typedef;

void GPS_Init(UART_HandleTypeDef*);

void GPS_Callback(UART_HandleTypeDef*, GPS_Data_Typedef*);

#endif /* INC_GPS_H_ */
