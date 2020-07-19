/* speedy.h 

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
#ifndef SPEEDY_H
#define SPEEDY_H

typedef struct {
    FIREPLACE private_fireplace;
} SPEEDY_STATE;

void speedy_init(SPEEDY_STATE *pstate);
void speedy_step(SPEEDY_STATE *pstate, SHARED_MEMORY *shmem);

#endif // SPEEDY_H