#include "cpu_utils.h"
#include "FreeRTOS.h"
#include "task.h"

#define CALCULATION_PERIOD    1000

#if 0 // How to enable cpu monitor
// 1 : Change definition in FreeRTOSConfig.h
    - #define configUSE_IDLE_HOOK        1
    - #define configUSE_TICK_HOOK        1

// 2 : Change definition in FreeRTOS.h
    - #define traceTASK_SWITCHED_IN()  extern void StartIdleMonitor(void); \
                                        StartIdleMonitor()
    - #define traceTASK_SWITCHED_OUT() extern void EndIdleMonitor(void); \
                                        EndIdleMonitor()
#endif

static xTaskHandle          m_cpu_handle = NULL;
static volatile uint32_t    m_cpu_usage = 0; 
static uint32_t             m_cpu_idle_start_time = 0; 
static uint32_t             m_cpu_idle_spent_time = 0; 
static uint32_t             m_cpu_total_idle_time = 0; 
static uint32_t             m_highest_cpu_use     = 0;

void vApplicationIdleHook(void) 
{
    if (m_cpu_handle == NULL)
    {
        m_cpu_handle = xTaskGetCurrentTaskHandle();
    }
}

void vApplicationTickHook(void)
{
    static uint32_t tick = 0;

    if (tick++ > CALCULATION_PERIOD)
    {
        tick = 0;
        
        if (m_cpu_total_idle_time > CALCULATION_PERIOD)
        {
            m_cpu_total_idle_time = CALCULATION_PERIOD;
        }

        m_cpu_usage = (100 - (m_cpu_total_idle_time * 100) / CALCULATION_PERIOD);
        if (m_highest_cpu_use < m_cpu_usage)
            m_highest_cpu_use = m_cpu_usage;
        
        m_cpu_total_idle_time = 0;
    }
}

void StartIdleMonitor(void)
{
    if (xTaskGetCurrentTaskHandle() == m_cpu_handle) 
    {
        m_cpu_idle_start_time = xTaskGetTickCountFromISR();
    }
}

void EndIdleMonitor(void)
{
    if (xTaskGetCurrentTaskHandle() == m_cpu_handle)
    {
        /* Store the handle to the idle task */
        m_cpu_idle_spent_time = xTaskGetTickCountFromISR() - m_cpu_idle_start_time;
        m_cpu_total_idle_time += m_cpu_idle_spent_time; 
    }
}

uint16_t cpu_get_usage (void)
{
    return (uint16_t)m_cpu_usage;
}

uint16_t cpu_get_highest_usage(void)
{
    return m_highest_cpu_use;
}
