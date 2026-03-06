#pragma once

#include <stdint.h>
#include "stm32f4xx.h"


#ifdef __cplusplus
extern "C"{
#endif

typedef struct{



}eldriver_core_t;

void eldriver_core_init(eldriver_core_t *h);


static inline uint32_t eldriver_core_prof_tick()
{
    return DWT->CYCCNT;
};

static inline uint32_t eldriver_core_prof_tock(uint32_t start)
{
    return (DWT->CYCCNT - start);
};

#ifdef __cplusplus
}
#endif