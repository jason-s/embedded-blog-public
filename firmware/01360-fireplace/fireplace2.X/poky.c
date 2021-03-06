/* poky.c 
 
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
#include "poky.h"
#include "mock_uart.h"

void poky_init(POKY_STATE *pstate, SHARED_MEMORY *shmem)
{
    poky_sum_init(pstate);
    fireplace_init(&shmem->fireplaces[0]);
    fireplace_init(&shmem->fireplaces[1]);
    shmem->access.poky   = &shmem->fireplaces[0];
    shmem->access.speedy = &shmem->fireplaces[1];
    shmem->switch_request = false;
}

void poky_step(POKY_STATE *pstate, SHARED_MEMORY *shmem)
{
    /* called during the main loop */

    // Skip if Speedy hasn't processed the switch request
    if (!shmem->switch_request)
    {
        volatile FIREPLACE *mine = shmem->access.poky;
        if (mine->count > 0)
        {
            // Speedy has accumulated samples! 
            // Let's accumulate that into an overall sum.
            pstate->voltage_sum += mine->voltage_sum;
            pstate->current_sum += mine->current_sum;
            pstate->count += mine->count;

            // Now zero out the accumulators so we can switch fireplaces next time.
            fireplace_init(mine);
        }
        shmem->switch_request = true;
    }

    // If we get a message asking for the sums, send them and zero them out.
    if (should_we_transmit_sums())
    {
        transmit_sums(pstate->voltage_sum, pstate->current_sum, pstate->count);
        poky_sum_init(pstate);
    }
}