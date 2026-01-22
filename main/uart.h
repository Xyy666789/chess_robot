#pragma once

void uart_init(void);
void rx_task(void* arg);
void uart_send(const char* data);

/*#ifndef UART_H
#define UART_H

#include <stdint.h>

void k210_uart_tx( char* packet_buffer,uint16_t buffered_size);
void uart_event_task(void *pvParameters);
void report_car_status_to_k210(void *pvParameters);

#endif */
