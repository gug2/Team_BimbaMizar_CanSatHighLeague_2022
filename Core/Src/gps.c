/*
 * LSM303DTR.c
 *
 *  Created on: May 8, 2022 г.
 *      Author: andre
 */
#include "gps.h"

static UART_HandleTypeDef* huart;

uint8_t receiveByte;
uint8_t gpsBuffer[GPS_BUFFER_SIZE];
uint8_t gpsIndex = 0;

void GPS_Init(UART_HandleTypeDef* huartx) {
	huart = huartx;

	// delay for correct device start
	HAL_Delay(50);

	HAL_UART_Receive_IT(huart, &receiveByte, 1);
}

// пример NMEA сообщения
// $GPGGA,132151.00,5312.79630,N,05010.19659,E,1,07,1.52,156.0,M,-6.4,M,,*4D
// $GPRMC,132151.00,A,5312.79630,N,05010.19659,E,1.250,336.95,170622,,,A*6F

uint8_t receiveData = 0;
uint8_t nmeaHeaderBuf[6];
uint8_t msgResult = 0;

uint8_t getMsg(GPS_Data_Typedef* obj) {
	uint8_t startI = 0, startOfPacket = 0;
	uint8_t checksum = 0;

	// go to last $ in msg
	for(uint8_t i = 0; i < gpsIndex; i++) {
		if(gpsBuffer[i] == '$') startI = i;
	}

	// skip $
	startI++;
	startOfPacket = startI;

	// calculate checksum
	while(gpsBuffer[startI] != '*' && gpsBuffer[startI] != '\0') {
		checksum ^= gpsBuffer[startI];
		startI++;
	}
	if(gpsBuffer[startI] == '\0') {
		// error, no * reached
		return 0;
	}

	// get msg checksum
	char msgChecksum[3];
	msgChecksum[0] = gpsBuffer[startI + 1];
	msgChecksum[1] = gpsBuffer[startI + 2];
	msgChecksum[2] = '\0';

	// convert calculated checksum to hex format
	char calcChecksum[3];
	sprintf(calcChecksum, "%02X", checksum);

	uint8_t isPacketValid = strcmp(calcChecksum, msgChecksum) == 0;

	if(isPacketValid) {
		sscanf(gpsBuffer + startOfPacket, "GPRMC,%f,%c,%d.%d,%c,%d.%d,%c", &obj->hhmmss_ms, &obj->packetValid, &obj->latitude1, &obj->latitude2, &obj->NS, &obj->longitude1, &obj->longitude2, &obj->EW);
	}

	return isPacketValid;

	/*for(uint8_t i = 0; i < 5; i++) {
		nmeaHeaderBuf[i] = gpsBuffer[startIndex++];
	}
	nmeaHeaderBuf[5] = '\0';

	if(strcmp((char*)nmeaHeaderBuf, "GPRMC") == 0) {

		return 1;
	}*/

	//return 0;
}

void GPS_Callback(UART_HandleTypeDef* callbackUart, GPS_Data_Typedef* obj) {
	if(callbackUart != huart) return;

	//if(!receiveData && receiveByte == '$') {
	//	receiveData = 1;
	//}

	//if(receiveData) {

	if(receiveByte != '\n' && gpsIndex < GPS_BUFFER_SIZE) {
		gpsBuffer[gpsIndex++] = receiveByte;
	} else {
		gpsBuffer[gpsIndex] = '\0';
		msgResult = getMsg(obj);

		receiveData = 0;
		gpsIndex = 0;
		memset(gpsBuffer, 0, GPS_BUFFER_SIZE);
	}

	//}

	/**if(receiveData && (receiveByte != '*' || gpsIndex < GPS_BUFFER_SIZE)) {
		gpsBuffer[gpsIndex++] = receiveByte;
	} else if(receiveByte == '*') {
		uint8_t test = 2;

		receiveData = 0;
		gpsIndex = 0;
		memset(gpsBuffer, 0, GPS_BUFFER_SIZE);
	} else if(gpsIndex >= GPS_BUFFER_SIZE) {
		uint8_t test2 = 2;

		receiveData = 0;
		gpsIndex = 0;
		memset(gpsBuffer, 0, GPS_BUFFER_SIZE);
	}**/

	/*if(receiveByte != '\n' && gpsIndex < GPS_BUFFER_SIZE) {
		gpsBuffer[gpsIndex++] = receiveByte;
	} else {
		uint8_t test = 2;

		gpsIndex = 0;
		memset(gpsBuffer, 0, GPS_BUFFER_SIZE);
	}*/
	HAL_UART_Receive_IT(huart, &receiveByte, 1);
}
