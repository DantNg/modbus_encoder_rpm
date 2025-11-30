/*
 * modbus_master.c
 *
 *  Created on: Jul 16, 2025
 *      Author: victus16
 */
#include "modbus_master.h"
#include "queue/queue.h"
#include "crc16/crc16.h"
#include <string.h>

static UART_HandleTypeDef *modbus_uart = NULL;
ModbusResponseCallback modbus_user_on_response = NULL;

void modbus_master_init(UART_HandleTypeDef *huart) {
    modbus_uart = huart;
}

bool modbus_master_send_request(ModbusRequest_t *req) {
    if (!modbus_uart) return false;

    uint8_t frame[256];
    uint16_t len = 0;

    frame[len++] = req->slave_id;
    frame[len++] = req->func_code;
    frame[len++] = req->addr >> 8;
    frame[len++] = req->addr & 0xFF;

    switch (req->func_code) {
        case MODBUS_FC_READ_COILS:
        case MODBUS_FC_READ_DISCRETE_INPUTS:
        case MODBUS_FC_READ_HOLDING_REGISTERS:
        case MODBUS_FC_READ_INPUT_REGISTERS:
            frame[len++] = req->quantity >> 8;
            frame[len++] = req->quantity & 0xFF;
            break;

        case MODBUS_FC_WRITE_SINGLE_COIL:
        case MODBUS_FC_WRITE_SINGLE_REGISTER:
            frame[len++] = req->write_data[0] >> 8;
            frame[len++] = req->write_data[0] & 0xFF;
            break;

        case MODBUS_FC_WRITE_MULTIPLE_COILS: {
            uint16_t byte_count = (req->quantity + 7) / 8;
            frame[len++] = req->quantity >> 8;
            frame[len++] = req->quantity & 0xFF;
            frame[len++] = byte_count;

            for (uint16_t i = 0; i < byte_count; i++) {
                uint8_t byte = 0;
                for (uint8_t b = 0; b < 8; b++) {
                    uint16_t bit_index = i * 8 + b;
                    if (bit_index < req->quantity && req->write_data[bit_index])
                        byte |= (1 << b);
                }
                frame[len++] = byte;
            }
            break;
        }

        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            frame[len++] = req->quantity >> 8;
            frame[len++] = req->quantity & 0xFF;
            frame[len++] = req->quantity * 2;

            for (uint16_t i = 0; i < req->quantity; i++) {
                frame[len++] = req->write_data[i] >> 8;
                frame[len++] = req->write_data[i] & 0xFF;
            }
            break;

        default:
            return false;
    }

    uint16_t crc = modbus_crc16(frame, len);
    frame[len++] = crc & 0xFF;
    frame[len++] = crc >> 8;

    return HAL_UART_Transmit_DMA(modbus_uart, frame, len) == HAL_OK;
}

void modbus_master_handle_response(uint8_t *data, uint16_t len) {
    if (len < 5) return;

    uint16_t crc_calc = modbus_crc16(data, len - 2);
    uint16_t crc_recv = data[len - 2] | (data[len - 1] << 8);
    if (crc_calc != crc_recv) return;

    if (modbus_user_on_response) {
        modbus_user_on_response(data, len);
    }
}
bool modbus_parse_response(uint8_t *data, uint16_t len, ModbusParsedResponse_t *out) {
    if (!data || len < 5 || !out) return false;

    memset(out, 0, sizeof(ModbusParsedResponse_t));
    out->slave_id = data[0];
    uint8_t func = data[1];

    if (func & 0x80) {
        out->is_exception = true;
        out->func_code = (ModbusFunctionCode)(func & 0x7F);
        out->exception_code = data[2];
        return true;
    }

    out->func_code = (ModbusFunctionCode)func;

    switch (func) {
        case MODBUS_FC_READ_HOLDING_REGISTERS:
        case MODBUS_FC_READ_INPUT_REGISTERS: {
            uint8_t byte_count = data[2];
            if (byte_count > sizeof(out->read.registers) * 2 || len < byte_count + 5)
                return false;

            out->read.byte_count = byte_count;
            out->read.quantity = byte_count / 2;

            for (uint16_t i = 0; i < out->read.quantity; i++) {
                out->read.registers[i] = (data[3 + i * 2] << 8) | data[4 + i * 2];
            }
            return true;
        }

        case MODBUS_FC_WRITE_SINGLE_REGISTER:
        case MODBUS_FC_WRITE_SINGLE_COIL:
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
        case MODBUS_FC_WRITE_MULTIPLE_COILS: {
            if (len < 8) return false;
            out->write_ack.addr = (data[2] << 8) | data[3];
            out->write_ack.value = (data[4] << 8) | data[5];
            return true;
        }

        default:
            return false;
    }
}
