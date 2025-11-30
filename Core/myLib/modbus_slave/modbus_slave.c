// modbus_slave.c
#include "modbus_slave.h"
#include "crc16/crc16.h"
#include <string.h>
#include <stdio.h>
static modbus_slave_config_t slave_cfg;
static UART_HandleTypeDef *modbus_uart;

void modbus_slave_init(UART_HandleTypeDef *huart, modbus_slave_config_t *cfg) {
	modbus_uart = huart;
	slave_cfg = *cfg;
}

void send_response(uint8_t *data, uint16_t len) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_UART_Transmit_DMA(modbus_uart, data, len);
}

void modbus_slave_handle_frame(const uint8_t *frame, uint16_t len) {
	if (len < 5)
		return;

	if (frame[0] != slave_cfg.id)
		return;

	uint16_t crc_recv = frame[len - 2] | (frame[len - 1] << 8);
	uint16_t crc_calc = modbus_crc16(frame, len - 2);
	if (crc_recv != crc_calc)
		return;

	modbus_function_code_t func = (modbus_function_code_t) frame[1];
	uint16_t addr = 0;
	uint16_t quantity = 0;
	uint8_t byte_count = 0;
	uint8_t response[256];
	printf("Request to %d with Function Code 0x%02X\n",frame[0],func);
	switch (func) {
	case MODBUS_FUNC_READ_COILS: // Read Coils
	{

		addr = (frame[2] << 8) | frame[3];
		quantity = (frame[4] << 8) | frame[5];
		if ((addr + quantity) > slave_cfg.coil_count)
			return;

		byte_count = (quantity + 7) / 8;
		response[0] = slave_cfg.id;
		response[1] = func;
		response[2] = byte_count;

		for (uint16_t i = 0; i < quantity; ++i) {
			if (slave_cfg.coils[addr + i])
				response[3 + i / 8] |= (1 << (i % 8));
			else
				response[3 + i / 8] &= ~(1 << (i % 8));
		}

		if (slave_cfg.on_read_coils)
			slave_cfg.on_read_coils(addr, quantity);

		uint16_t crc = modbus_crc16(response, 3 + byte_count);
		response[3 + byte_count] = crc & 0xFF;
		response[4 + byte_count] = crc >> 8;

		send_response(response, 5 + byte_count);

	}
		break;
	case MODBUS_FUNC_READ_DISCRETE_INPUTS: // Read Discrete Inputs
	{
		addr = (frame[2] << 8) | frame[3];
		quantity = (frame[4] << 8) | frame[5];
		if ((addr + quantity) > slave_cfg.discrete_input_count)
			return;

		byte_count = (quantity + 7) / 8;
		response[0] = slave_cfg.id;
		response[1] = func;
		response[2] = byte_count;

		for (uint16_t i = 0; i < quantity; ++i) {
			if (slave_cfg.discrete_inputs[addr + i])
				response[3 + i / 8] |= (1 << (i % 8));
			else
				response[3 + i / 8] &= ~(1 << (i % 8));
		}

		if (slave_cfg.on_read_discrete_inputs)
			slave_cfg.on_read_discrete_inputs(addr, quantity);

		uint16_t crc = modbus_crc16(response, 3 + byte_count);
		response[3 + byte_count] = crc & 0xFF;
		response[4 + byte_count] = crc >> 8;

		send_response(response, 5 + byte_count);
	}
		break;
	case MODBUS_FUNC_READ_HOLDING_REGISTERS: { // Read Holding Registers
		if (len != 8)
			return;
		uint16_t addr = (frame[2] << 8) | frame[3];
		uint16_t count = (frame[4] << 8) | frame[5];
		if (addr + count > slave_cfg.holding_register_count || count == 0)
			return;
		if (slave_cfg.on_read_holding_registers) {
			slave_cfg.on_read_holding_registers(addr, count);
		}
		uint8_t resp[256];
		resp[0] = slave_cfg.id;
		resp[1] = func;
		resp[2] = count * 2;
		for (uint16_t i = 0; i < count; i++) {
			resp[3 + i * 2] = slave_cfg.holding_registers[addr + i] >> 8;
			resp[4 + i * 2] = slave_cfg.holding_registers[addr + i] & 0xFF;
		}

		uint16_t crc = modbus_crc16(resp, 3 + count * 2);
		resp[3 + count * 2] = crc & 0xFF;
		resp[4 + count * 2] = crc >> 8;

		send_response(resp, 5 + count * 2);
		break;
	}
	case MODBUS_FUNC_READ_INPUT_REGISTERS: // Read Input Registers
	{
		addr = (frame[2] << 8) | frame[3];
		quantity = (frame[4] << 8) | frame[5];
		if ((addr + quantity) > slave_cfg.input_register_count)
			return;

		byte_count = quantity * 2;
		response[0] = slave_cfg.id;
		response[1] = func;
		response[2] = byte_count;

		for (uint16_t i = 0; i < quantity; ++i) {
			response[3 + 2 * i] = slave_cfg.input_registers[addr + i] >> 8;
			response[3 + 2 * i + 1] = slave_cfg.input_registers[addr + i]
					& 0xFF;
		}

		if (slave_cfg.on_read_input_registers)
			slave_cfg.on_read_input_registers(addr, quantity);

		uint16_t crc = modbus_crc16(response, 3 + byte_count);
		response[3 + byte_count] = crc & 0xFF;
		response[4 + byte_count] = crc >> 8;

		send_response(response, 5 + byte_count);
	}
		break;
	case MODBUS_FUNC_WRITE_SINGLE_COIL: // Write Single Coil
	{
		addr = (frame[2] << 8) | frame[3];
		uint16_t value = (frame[4] << 8) | frame[5];
		if (addr >= slave_cfg.coil_count)
			return;

		slave_cfg.coils[addr] = (value == 0xFF00) ? 1 : 0;

		if (slave_cfg.on_write_single_coil)
			slave_cfg.on_write_single_coil(addr, slave_cfg.coils[addr]);
		memcpy(response, frame, 6);  // Echo request
		uint16_t crc = modbus_crc16(response, 6);
		response[3 + byte_count] = crc & 0xFF;
		response[4 + byte_count] = crc >> 8;

		send_response(response, 8);
	}
		break;

	case MODBUS_FUNC_WRITE_SINGLE_REGISTER: { // Write Single Register
		if (len != 8)
			return;
		uint16_t addr = (frame[2] << 8) | frame[3];
		uint16_t val = (frame[4] << 8) | frame[5];
		if (addr >= slave_cfg.holding_register_count)
			return;

		slave_cfg.holding_registers[addr] = val;
		if (slave_cfg.on_write_single_register) {
			slave_cfg.on_write_single_register(addr, val);
		}

		// Echo original frame
		send_response((uint8_t*) frame, len);
		break;
	}
	case MODBUS_FUNC_WRITE_MULTIPLE_COILS: // Write Multiple Coils
	{
		addr = (frame[2] << 8) | frame[3];
		quantity = (frame[4] << 8) | frame[5];
		byte_count = frame[6];
		if ((addr + quantity) > slave_cfg.coil_count)
			return;

		for (uint16_t i = 0; i < quantity; ++i) {
			uint8_t bit = (frame[7 + i / 8] >> (i % 8)) & 0x01;
			slave_cfg.coils[addr + i] = bit;
		}

		if (slave_cfg.on_write_multiple_coils)
			slave_cfg.on_write_multiple_coils(addr, &frame[7], quantity);

		response[0] = slave_cfg.id;
		response[1] = 0x0F;
		response[2] = frame[2];
		response[3] = frame[3];
		response[4] = frame[4];
		response[5] = frame[5];

		uint16_t crc = modbus_crc16(response, 6);
		response[3 + byte_count] = crc & 0xFF;
		response[4 + byte_count] = crc >> 8;

		send_response(response, 8);

	}
		break;
	case MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS: { // Write Multiple Registers
		if (len < 9)
			return;
		uint16_t addr = (frame[2] << 8) | frame[3];
		uint16_t count = (frame[4] << 8) | frame[5];
		uint8_t byte_count = frame[6];

		if (addr + count > slave_cfg.holding_register_count)
			return;
		if (byte_count != count * 2 || len != 9 + byte_count)
			return;

		for (uint16_t i = 0; i < count; i++) {
			uint16_t val = (frame[7 + i * 2] << 8) | frame[8 + i * 2];
			slave_cfg.holding_registers[addr + i] = val;
		}

		if (slave_cfg.on_write_multiple_registers) {
			slave_cfg.on_write_multiple_registers(addr,
					&slave_cfg.holding_registers[addr], count);
		}

		uint8_t resp[8];
		memcpy(resp, frame, 6);
		uint16_t crc = modbus_crc16(resp, 6);
		resp[6] = crc & 0xFF;
		resp[7] = crc >> 8;

		send_response(resp, 8);
		break;
	}
	default:
		// unsupported function code
		break;
	}
}
