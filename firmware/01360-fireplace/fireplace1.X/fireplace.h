/* fireplace.h */
#ifndef FIREPLACE_H
#define FIREPLACE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int32_t voltage_sum;
    int32_t current_sum;
    uint16_t count;
} FIREPLACE;

typedef struct {
    FIREPLACE fireplaces[2];
    struct {
        volatile FIREPLACE *speedy;
        volatile FIREPLACE *poky;
    } access;
} RAW_SHARED_MEMORY; // w/o volatile -- do not use directly

typedef volatile RAW_SHARED_MEMORY SHARED_MEMORY;

inline static void fireplace_init(volatile FIREPLACE *fp)
{
    fp->voltage_sum = 0;
    fp->current_sum = 0;
    fp->count = 0;
}

inline static void fireplace_switch(SHARED_MEMORY *shmem)
{
    volatile FIREPLACE *tmp = shmem->access.poky;
    shmem->access.poky = shmem->access.speedy;
    shmem->access.speedy = tmp;
}

#endif // FIREPLACE_H