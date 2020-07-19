/* speedy.h */
#ifndef SPEEDY_H
#define SPEEDY_H

typedef struct {
    FIREPLACE private_fireplace;
} SPEEDY_STATE;

void speedy_init(SPEEDY_STATE *pstate);
void speedy_step(SPEEDY_STATE *pstate, SHARED_MEMORY *shmem);

#endif // SPEEDY_H