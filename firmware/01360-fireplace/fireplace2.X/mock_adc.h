/* mock_adc */
#ifndef MOCK_ADC_H
#define MOCK_ADC_H

#include <stdint.h>

typedef enum {
    ADC_CURRENT = 0,
    ADC_VOLTAGE = 1
} ADC_CHANNEL_T;

typedef struct {
    uint32_t state;
    uint32_t a;
    uint32_t c;
} MOCK_ADC_STATE_T;

void init_mock_adc(MOCK_ADC_STATE_T *adcstate, uint32_t s);
int16_t read_mock_adc(MOCK_ADC_STATE_T *adcstate);
void init_adc();
int16_t read_adc(ADC_CHANNEL_T channel);


#endif // MOCK_ADC_H