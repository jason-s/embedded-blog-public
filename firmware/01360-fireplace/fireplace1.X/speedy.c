/* speedy.c 
 
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