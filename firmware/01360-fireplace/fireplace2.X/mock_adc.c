#include "mock_adc.h"

MOCK_ADC_STATE_T adcstate[2];

// see https://en.wikipedia.org/wiki/Linear_congruential_generator#Parameters_in_common_use
#define LCG_A 1664525
#define LCG_C 1013904223UL

void init_mock_adc(MOCK_ADC_STATE_T *adcstate, uint32_t s)
{
    adcstate->a = LCG_A;
    adcstate->c = LCG_C;
    adcstate->state = s;    
}

int16_t read_mock_adc(MOCK_ADC_STATE_T *adcstate)
{
    adcstate->state = adcstate->state * adcstate->a + adcstate->c;
    return adcstate->state;
}

void init_adc() {
    init_mock_adc(&adcstate[0], 0);
    init_mock_adc(&adcstate[1], 1);
}

int16_t read_adc(ADC_CHANNEL_T channel)
{
    return read_mock_adc(&adcstate[channel]);
}
