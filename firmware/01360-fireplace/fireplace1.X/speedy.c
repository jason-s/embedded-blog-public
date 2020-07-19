/* speedy.c */

#include "fireplace.h"
#include "mock_adc.h"

void speedy_step(SHARED_MEMORY *shmem)
{
    // read ADC
    int16_t current = read_adc(ADC_CURRENT);
    int16_t voltage = read_adc(ADC_VOLTAGE);

    volatile FIREPLACE *mine = shmem->access.speedy;
    mine->voltage_sum += voltage;
    mine->current_sum += current;
    ++mine->count;
}