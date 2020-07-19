/* poky.h

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
#ifndef POKY_H
#define POKY_H

#include "fireplace.h"

typedef struct {
    int64_t voltage_sum;
    int64_t current_sum;
    uint32_t count;
} POKY_STATE;

inline static void poky_sum_init(POKY_STATE *pstate)
{
    pstate->voltage_sum = 0;
    pstate->current_sum = 0;
    pstate->count = 0;
}

void poky_init(POKY_STATE *pstate, SHARED_MEMORY *shmem);
void poky_step(POKY_STATE *pstate, SHARED_MEMORY *shmem);

#endif // POKY_H