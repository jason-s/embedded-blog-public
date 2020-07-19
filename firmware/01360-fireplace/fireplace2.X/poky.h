/* poky.h */
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