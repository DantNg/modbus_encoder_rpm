/*
 * queue.h
 *
 *  Created on: Jul 16, 2025
 *      Author: victus16
 */

#ifndef MODBUS_QUEUE_QUEUE_H_
#define MODBUS_QUEUE_QUEUE_H_
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#define QUEUE_SIZE     8
#define MODBUS_FRAME_MAX_LEN  256
#define UART_RX_BUFFER_SIZE 256
extern uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
typedef struct {
    uint8_t data[MODBUS_FRAME_MAX_LEN];
    uint16_t len;
} queue_frame_t;

void queue_init(void);
bool queue_push(queue_frame_t *frame);
bool queue_pop(queue_frame_t *out_frame);
bool queue_is_empty(void);

#endif /* MODBUS_QUEUE_QUEUE_H_ */
