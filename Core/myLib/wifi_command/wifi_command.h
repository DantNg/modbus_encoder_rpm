// wifi_command.h
#ifndef WIFI_COMMAND_H
#define WIFI_COMMAND_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define WIFI_CMD_RX_BUF_SIZE 64
#define WIFI_RX_BUFFER_SIZE 64
extern uint8_t wifi_rx_dma_buf[WIFI_RX_BUFFER_SIZE];

typedef enum {
    CMD_TYPE_INT,
    CMD_TYPE_DOUBLE,
    CMD_TYPE_STRING,
    CMD_TYPE_WORD,
    CMD_TYPE_DWORD
} wifi_cmd_type_t;

typedef struct {
    char name[16];
    wifi_cmd_type_t type;
    union {
        int i_val;
        double d_val;
        char s_val[32];
        uint16_t w_val;
        uint32_t dw_val;
    } value;
} wifi_command_t;

typedef struct {
    UART_HandleTypeDef *uart;
    char rx_buffer[WIFI_CMD_RX_BUF_SIZE];
    uint16_t rx_index;
    void (*on_command)(const wifi_command_t *cmd);
} wifi_command_handler_t;

void wifi_command_init(wifi_command_handler_t *handler, UART_HandleTypeDef *uart, void (*callback)(const wifi_command_t *));
bool wifi_command_parse(const char *input, wifi_command_t *out_cmd);

#endif // WIFI_COMMAND_H
