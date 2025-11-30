#ifndef MY_UART_H
#define MY_UART_H

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define UART_BUFFER_SIZE 64

typedef struct {
    UART_HandleTypeDef* huart;
    char rx_buffer[UART_BUFFER_SIZE];
    uint8_t index;

    uint8_t has_command;
    char command[8];
    char value[32];
} UartReceiver_t;

// Khởi tạo UART receiver
static inline void MyUart_Init(UartReceiver_t* uart, UART_HandleTypeDef* huart) {
    uart->huart = huart;
    uart->index = 0;
    uart->has_command = 0;
    memset(uart->rx_buffer, 0, UART_BUFFER_SIZE);
    memset(uart->command, 0, sizeof(uart->command));
    memset(uart->value, 0, sizeof(uart->value));
}

// Gọi trong HAL_UART_RxCpltCallback
static inline void MyUart_OnRx(UartReceiver_t* uart, uint8_t byte) {
    if (byte == '\r' || byte == '\n') {
        uart->rx_buffer[uart->index] = '\0';
        uart->index = 0;

        // Tách command=value
        char* equal = strchr(uart->rx_buffer, '=');
        if (equal) {
            size_t cmd_len = equal - uart->rx_buffer;
            if (cmd_len < sizeof(uart->command)) {
                strncpy(uart->command, uart->rx_buffer, cmd_len);
                uart->command[cmd_len] = '\0';
                strncpy(uart->value, equal + 1, sizeof(uart->value) - 1);
                uart->has_command = 1;
            }
        }

        memset(uart->rx_buffer, 0, UART_BUFFER_SIZE);
    } else if (uart->index < UART_BUFFER_SIZE - 1) {
        uart->rx_buffer[uart->index++] = byte;
    }
}

#endif // MY_UART_H
