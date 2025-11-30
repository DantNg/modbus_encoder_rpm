// wifi_command.c
#include "wifi_command.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
uint8_t wifi_rx_dma_buf[WIFI_RX_BUFFER_SIZE];
void wifi_command_init(wifi_command_handler_t *handler,
		UART_HandleTypeDef *uart, void (*callback)(const wifi_command_t*)) {
	if (!handler)
		return;
	handler->uart = uart;
	handler->rx_index = 0;
	handler->on_command = callback;
}
static void trim_end(char *str) {
	for (int i = 0; str[i]; i++) {
		if (str[i] == '\r' || str[i] == '\n') {
			str[i] = '\0';
			break;
		}
	}
}

bool wifi_command_parse(const char *input, wifi_command_t *out_cmd) {
	if (!input || !out_cmd)
		return false;

	char name[16], value_str[32];
	if (sscanf(input, "%15s %31s", name, value_str) != 2)
		return false;

	strncpy(out_cmd->name, name, sizeof(out_cmd->name));

	// Command mapping - customize as needed
	if (strcmp(name, "DIAMETER") == 0) {
		out_cmd->type = CMD_TYPE_WORD;
		out_cmd->value.w_val = (uint16_t) atoi(value_str);
	} else if (strcmp(name, "SPEED") == 0) {
		out_cmd->type = CMD_TYPE_DOUBLE;
		trim_end(value_str);
		out_cmd->value.d_val = atof(value_str);
	} else if (strcmp(name, "NAME") == 0) {
		out_cmd->type = CMD_TYPE_STRING;
		strncpy(out_cmd->value.s_val, value_str, sizeof(out_cmd->value.s_val));
	} else if (strcmp(name, "DIST") == 0) {
		out_cmd->type = CMD_TYPE_DWORD;
		out_cmd->value.dw_val = (uint32_t) strtoul(value_str, NULL, 10);
	} else if (strcmp(name, "GAIN") == 0) {
		out_cmd->type = CMD_TYPE_INT;
		out_cmd->value.i_val = atoi(value_str);
	} else {
		return false; // unknown command
	}

	return true;
}
