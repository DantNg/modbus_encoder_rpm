/*
 * queue.c
 *
 *  Created on: Jul 16, 2025
 *      Author: datnguyen
 */
#include "queue.h"
#include <string.h>

static queue_frame_t queue[QUEUE_SIZE];
static volatile uint8_t head = 0;
static volatile uint8_t tail = 0;
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
void queue_init(void)
{
    head = 0;
    tail = 0;
}

bool queue_push(queue_frame_t *frame)
{
    uint8_t next = (head + 1) % QUEUE_SIZE;
    if (next == tail) return false; // full

    memcpy(&queue[head], frame, sizeof(queue_frame_t));
    head = next;
    return true;
}

bool queue_pop(queue_frame_t *out_frame)
{
    if (head == tail) return false; // empty

    memcpy(out_frame, &queue[tail], sizeof(queue_frame_t));
    tail = (tail + 1) % QUEUE_SIZE;
    return true;
}

bool queue_is_empty(void)
{
    return head == tail;
}
