/*
 * crc16.h
 *
 *  Created on: Jul 16, 2025
 *      Author: datnguyen
 */

#ifndef MODBUS_MODBUS_MASTER_CRC16_H_
#define MODBUS_MODBUS_MASTER_CRC16_H_
#include <stdint.h>
#include <stdbool.h>


uint16_t modbus_crc16(const uint8_t *data, uint16_t len);

#endif /* MODBUS_MODBUS_MASTER_CRC16_H_ */
