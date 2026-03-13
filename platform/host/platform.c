/* ---- Required hooks for the selected FreeRTOS POSIX configuration ---- */
#include "platform.h"
#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

void vAssertCalled(const char *pcFile, unsigned long ulLine)
{
    fprintf(stderr, "FreeRTOS assert: %s:%lu\n", pcFile, ulLine);
    fflush(stderr);
    abort();
}

void vApplicationMallocFailedHook(void)
{
    fprintf(stderr, "FreeRTOS malloc failed\n");
    fflush(stderr);
    abort();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    fprintf(stderr, "FreeRTOS stack overflow in task: %s\n", pcTaskName ? pcTaskName : "(unknown)");
    fflush(stderr);
    abort();
}

void vApplicationIdleHook(void)
{
}

void vApplicationTickHook(void)
{
}

void vApplicationDaemonTaskStartupHook(void)
{
}

#if (configSUPPORT_STATIC_ALLOCATION == 1)
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   StackType_t *pulIdleTaskStackSize)
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    StackType_t *pulTimerTaskStackSize)
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

#endif


#include <stdint.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#define VTIM_TICK_HZ 1000000  // 100 kHz virtual tick resolution
volatile uint64_t virtual_tick = 0;
float vtime = 0;
vtimer_manager_t timer_manager;

// get monotonic time in ns
static inline uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

// register a timer (frequency in Hz)
bool register_timer(vtimer_manager_t* mgr, timer_callback_t cb, uint64_t timestep_ns) {
    if (mgr->timer_index >= HOST_TIMERS) return false;

    vtimer_t* t = &mgr->timers[mgr->timer_index++];
    t->periodic_time_ns = (timestep_ns);
    t->last_time_ns = now_ns();
    t->cb = cb;

    if(timestep_ns < mgr->min_timestep_ns)
    {
        mgr->min_timestep_ns = timestep_ns;
    }

    return true;
}

void* tick_thread(void* arg)
{
    const uint64_t sleep_ns = 1000000ULL;   // 1 ms coarse sleep
    uint64_t sim_time_ns = now_ns();        // simulation time anchored to wall clock
    uint64_t last_wall_ns = sim_time_ns;

    // determine smallest timestep needed by any timer
    uint64_t dt_ns = timer_manager.min_timestep_ns; // example 10 us, can be min of all timers

    while (1)
    {
        dt_ns = timer_manager.min_timestep_ns;
        uint64_t current_wall_ns = now_ns();
        uint64_t elapsed_ns = current_wall_ns - last_wall_ns;
        last_wall_ns = current_wall_ns;

        uint64_t steps = elapsed_ns / dt_ns;
        for (uint64_t i = 0; i < steps; i++)
        {
            sim_time_ns += dt_ns;

            // unified check of all timers
            for (int t_idx = 0; t_idx < timer_manager.timer_index; t_idx++)
            {
                vtimer_t* t = &timer_manager.timers[t_idx];
                if (!t->cb) continue;

                if (sim_time_ns - t->last_time_ns >= t->periodic_time_ns)
                {
                    t->last_time_ns += t->periodic_time_ns; // advance by one period
                    t->cb();
                }
            }
        }

        // coarse sleep to avoid spinning CPU
        struct timespec req = {0, sleep_ns};
        nanosleep(&req, NULL);
    }
}

void platform_init()
{
    timer_manager.min_timestep_ns = 1000000;
    pthread_t t;
    pthread_create(&t, NULL, tick_thread, NULL);
}