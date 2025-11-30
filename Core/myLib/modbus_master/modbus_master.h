/*
 * modbus_master.h
 *
 *  Created on: Jul 16, 2025
 *      Author: victus16
 */

#ifndef MODBUS_MODBUS_MASTER_MODBUS_MASTER_H_
#define MODBUS_MODBUS_MASTER_MODBUS_MASTER_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define MODBUS_MAX_FRAME_SIZE 256

typedef enum {
    MODBUS_FC_READ_COILS = 0x01,
    MODBUS_FC_READ_DISCRETE_INPUTS = 0x02,
    MODBUS_FC_READ_HOLDING_REGISTERS = 0x03,
    MODBUS_FC_READ_INPUT_REGISTERS = 0x04,
    MODBUS_FC_WRITE_SINGLE_COIL = 0x05,
    MODBUS_FC_WRITE_SINGLE_REGISTER = 0x06,
    MODBUS_FC_WRITE_MULTIPLE_COILS = 0x0F,
    MODBUS_FC_WRITE_MULTIPLE_REGISTERS = 0x10,
} ModbusFunctionCode;

typedef struct {
    uint8_t slave_id;
    ModbusFunctionCode func_code;
    uint16_t addr;
    uint16_t quantity;
    const uint16_t *write_data;
} ModbusRequest_t;

typedef struct {
    uint8_t slave_id;
    ModbusFunctionCode func_code;
    bool is_exception;
    uint8_t exception_code;

    union {
        struct {
            uint8_t byte_count;
            uint16_t registers[64]; // max 64 words
            uint16_t quantity;
        } read;

        struct {
            uint16_t addr;
            uint16_t value;
        } write_ack;
    };
} ModbusParsedResponse_t;

typedef void (*ModbusResponseCallback)(uint8_t *data, uint16_t len);

void modbus_master_init(UART_HandleTypeDef *huart);
bool modbus_master_send_request(ModbusRequest_t *req);
void modbus_master_handle_response(uint8_t *data, uint16_t len);
// modbus


bool modbus_parse_response(uint8_t *data, uint16_t len, ModbusParsedResponse_t *out);
#endif /* MODBUS_MODBUS_MASTER_MODBUS_MASTER_H_ */
