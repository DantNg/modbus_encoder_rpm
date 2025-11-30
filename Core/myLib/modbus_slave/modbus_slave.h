/*
 * modbus_slave.h
 *
 *  Created on: Jul 16, 2025
 *      Author: victus16
 */

#ifndef MODBUS_MODBUS_SLAVE_MODBUS_SLAVE_H_
#define MODBUS_MODBUS_SLAVE_MODBUS_SLAVE_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
typedef enum {
	MODBUS_FUNC_READ_COILS = 0x01, // Read Coils
	MODBUS_FUNC_READ_DISCRETE_INPUTS = 0x02, // Read Discrete Inputs
	MODBUS_FUNC_READ_HOLDING_REGISTERS = 0x03, // Read Holding Registers
	MODBUS_FUNC_READ_INPUT_REGISTERS = 0x04, // Read Input Registers
	MODBUS_FUNC_WRITE_SINGLE_COIL = 0x05, // Write Single Coil
	MODBUS_FUNC_WRITE_SINGLE_REGISTER = 0x06, // Write Single Register
	MODBUS_FUNC_WRITE_MULTIPLE_COILS = 0x0F, // Write Multiple Coils
	MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS = 0x10  // Write Multiple Registers
} modbus_function_code_t;
typedef struct {
    uint8_t id;
    uint8_t  *coils;              // 0x01, 0x05, 0x0F
    uint16_t coil_count;

    uint8_t  *discrete_inputs;    // 0x02
    uint16_t discrete_input_count;

    uint16_t *holding_registers;  // 0x03, 0x06, 0x10
    uint16_t holding_register_count;

    uint16_t *input_registers;    // 0x04
    uint16_t input_register_count;

    // Callback cho từng function code
    void (*on_read_coils)(uint16_t addr, uint16_t quantity);
    void (*on_read_discrete_inputs)(uint16_t addr, uint16_t quantity);
    void (*on_read_holding_registers)(uint16_t addr, uint16_t quantity);
    void (*on_read_input_registers)(uint16_t addr, uint16_t quantity);
    void (*on_write_single_coil)(uint16_t addr, bool value);
    void (*on_write_single_register)(uint16_t addr, uint16_t value);
    void (*on_write_multiple_coils)(uint16_t addr, const uint8_t *values, uint16_t quantity);
    void (*on_write_multiple_registers)(uint16_t addr, const uint16_t *values, uint16_t quantity);

} modbus_slave_config_t;

void modbus_slave_init(UART_HandleTypeDef *huart, modbus_slave_config_t *cfg);
void modbus_slave_handle_frame(const uint8_t *frame, uint16_t len);

#endif /* MODBUS_MODBUS_SLAVE_MODBUS_SLAVE_H_ */
