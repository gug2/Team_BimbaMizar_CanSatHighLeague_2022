/*
 * Databuf.c
 *
 *  Created on: May 13, 2022
 *      Author: andre
 */
#include "Databuf.h"

static uint8_t databuf[128] = {0};
static uint16_t datasize = 0;

uint8_t* Buf_getDatabuf() {
	return databuf;
}

uint16_t Buf_getDatasize() {
	return datasize;
}

/* Clear the data buffer */
void Buf_clear() {
	for(uint16_t i = 0; i < datasize; i++) databuf[i] = 0;

	datasize = 0;
}

/* Push byte to data buffer (LSB first) */
void Buf_push8(uint8_t byte) {
	databuf[datasize] = byte;
	datasize++;
}

/* Push two bytes to data buffer (LSB first) */
void Buf_push16(int16_t twoBytes) {
	// tb & 0x00FF   FF << 0
	// tb & 0xFF00   FF << 8
	uint8_t offset = 0;
	for(uint8_t i = 0; i < 2; i++) {
		offset = 8*i;
		databuf[datasize] = (uint8_t)((twoBytes & (0xFF << offset)) >> offset);
		datasize++;
	}
}

/* Push four bytes to data buffer (LSB first) */
void Buf_push32(uint32_t bytes) {
	uint8_t offset = 0;
	for(uint8_t i = 0; i < 4; i++) {
		offset = 8*i;
		databuf[datasize] = (uint8_t)((bytes & (0xFF << offset)) >> offset);
		datasize++;
	}
}

float int16_tToFloat(int16_t twoBytes, uint16_t multiplier) {
	return twoBytes / 32768.0F * multiplier;
}
