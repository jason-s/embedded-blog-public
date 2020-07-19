/* fireplace.h 

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

    bool switch_request;  
    // only Poky is allowed to set, 
    // only Speedy is allowed to clear
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