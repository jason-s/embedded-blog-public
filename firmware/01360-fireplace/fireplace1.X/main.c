#include <xc.h>
#include <stdbool.h>
#include "poky.h"
#include "speedy.h"
#include "mock_adc.h"
#include "mock_uart.h"

/*

Copyright 2020 Jason M. Sachs

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

*/

POKY_STATE poky_state;
SHARED_MEMORY shmem;
volatile int16_t something = 0;

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    speedy_step(&shmem);
    IFS0bits.T1IF = 0; // acknowledge timer interrupt
}

void initialize(void)
{
    // init hardware
    
    /* NOTE: This has NOT been completely configured
     * for real hardware -- there are no config bits
     * set, and the device PLL has not been initialized.
     * 
     * I have been testing just on the simulator.
     */
    
    // Configure Timer1 for period of 1000
    PR1 = 999;
    IEC0bits.T1IE = 1;
    INTCON2bits.GIE = 1;
    
        
    // init software    
    poky_init(&poky_state, &shmem);
    init_adc();
    init_mock_uart();
    
    // start timer!
    T1CONbits.TON = 1;
}

volatile uint16_t iter_count = 0;
volatile uint16_t iter_count_end = 500;

void verify_result(void)
{
    int64_t voltage_sum;
    int64_t current_sum;
    uint32_t count;
    
    get_transmitted_sums(&voltage_sum,
                         &current_sum,
                         &count);
    ++something; 
    
    MOCK_ADC_STATE_T adcV, adcI;
    init_mock_adc(&adcI, 0);
    init_mock_adc(&adcV, 1);
        ++something; 
    uint32_t k;
    int64_t voltage_sum_expected = 0;
    int64_t current_sum_expected = 0;
    for (k = 0; k < count; ++k)
    {
        voltage_sum_expected += read_mock_adc(&adcV);
        current_sum_expected += read_mock_adc(&adcI);
    }

    //  put a breakpoint on the following line
    ++something; 
    __builtin_nop();
    __builtin_software_breakpoint();
    
    volatile int optimizer_defeater = voltage_sum_expected + current_sum_expected;
}

uint32_t close_collisions = 0;

void main_loop_step(void)
{
    int k;
    
    int ndelay;
    
    // somewhat erratic delays
    // (mean of 1500 per iteration)
    switch (iter_count & 7)
    {
        case 0:
            ndelay = 7500;
            break;
        case 2:
            ndelay = 4000;
            break;
        case 5:
            ndelay = 400;
            break;
        case 6:
            ndelay = 100;
            break;
        default:
            ndelay = 0;
            break;
    }
    
    for (k = 0; k < ndelay; ++k)
    {
        __builtin_nop();
    }
    
    ++iter_count;
    bool end = iter_count == iter_count_end;
    set_mock_uart_transmit_ok(end);
    
    // Point A
    uint16_t nspeedy0 = shmem.access.speedy->count;
    volatile FIREPLACE *pspeedy_fp_before = shmem.access.speedy;
    poky_step(&poky_state, &shmem);
    
    if (shmem.access.speedy != pspeedy_fp_before)
    {
        // fireplaces have swapped 
        // let's see what's happened since Point A
        if (shmem.access.speedy->count > 0 || shmem.access.poky->count > nspeedy0)
            ++close_collisions;
    }
    
    if (end)
    {
        ++something;
        verify_result();
    }
}

int main(void) {
    initialize();
    while (true)
    {
        main_loop_step();
    }
    return 0;
}
