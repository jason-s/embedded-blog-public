/* mock_uart.h */
#ifndef MOCK_UART_H
#define MOCK_UART_H

#include <stdbool.h>
#include <stdint.h>

void init_mock_uart(void);
void set_mock_uart_transmit_ok(bool ok);
bool should_we_transmit_sums(void);
void transmit_sums(
    int64_t voltage_sum,
    int64_t current_sum,
    uint32_t count);
void get_transmitted_sums(
    int64_t *pvoltage_sum,
    int64_t *pcurrent_sum,
    uint32_t *pcount);

#endif // MOCK_UART_H