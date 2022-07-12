/*
 * Databuf.h
 *
 *  Created on: May 13, 2022
 *      Author: andre
 */

#ifndef INC_DATABUF_H_
#define INC_DATABUF_H_

#include <stdint.h>

uint8_t* Buf_getDatabuf();
uint16_t Buf_getDatasize();

void Buf_clear();
void Buf_push8(uint8_t);
void Buf_push16(int16_t);
void Buf_push32(uint32_t);

uint8_t Buf_pop8();
int16_t Buf_pop16();
uint32_t Buf_pop32();

float int16_tToFloat(int16_t, uint16_t);

#endif /* INC_DATABUF_H_ */
