/*
 * crc16.c
 *
 *  Created on: Jul 16, 2025
 *      Author: victus16
 */
#include "crc16.h"

static uint16_t crc16_update(uint16_t crc, uint8_t a) {
	for (int i = 0; i < 8; ++i) {
		if ((crc ^ a) & 1)
			crc = (crc >> 1) ^ 0xA001;
		else
			crc = (crc >> 1);
		a >>= 1;
	}
	return crc;
}
uint16_t modbus_crc16(const uint8_t *data, uint16_t len) {
	uint16_t crc = 0xFFFF;

	for (uint16_t i = 0; i < len; i++) {
		crc = crc16_update(crc, data[i]);
	}

	return crc;
}
