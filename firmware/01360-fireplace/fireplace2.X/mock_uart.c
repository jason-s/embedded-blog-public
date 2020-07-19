#include "mock_uart.h"

struct mock_uart_state_t
{
    bool transmit_ok;
    int64_t voltage_sum;
    int64_t current_sum;
    uint32_t count;
} mock_uart_state;

void init_mock_uart(void)
{
    mock_uart_state.transmit_ok = false;  
    mock_uart_state.voltage_sum = 0;
    mock_uart_state.current_sum = 0;
    mock_uart_state.count = 0;
}
void set_mock_uart_transmit_ok(bool ok)
{
    mock_uart_state.transmit_ok = ok;
}

bool should_we_transmit_sums(void)
{
    return mock_uart_state.transmit_ok;
}
void transmit_sums(
    int64_t voltage_sum,
    int64_t current_sum,
    uint32_t count)
{
    mock_uart_state.voltage_sum = voltage_sum;
    mock_uart_state.current_sum = current_sum;
    mock_uart_state.count = count;
}
void get_transmitted_sums(
    int64_t *pvoltage_sum,
    int64_t *pcurrent_sum,
    uint32_t *pcount)
{
    *pvoltage_sum = mock_uart_state.voltage_sum;
    *pcurrent_sum = mock_uart_state.current_sum;
    *pcount = mock_uart_state.count;
}