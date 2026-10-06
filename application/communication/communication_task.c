#include "communication_task.h"

#include "bsp_uart.h"
#include "cmsis_os.h"
#include "communication.h"
#include "vofa.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t communication_high_water;
#endif

// 任务相关时间
#define COMMUNICATION_TASK_INIT_TIME 100
#define COMMUNICATION_TASK_TIME_MS 2

void communication_task(void const * pvParameters)
{
#if VOFA_ENABLE
    TickType_t last_wake;
    const TickType_t period = pdMS_TO_TICKS(VOFA_PERIOD_MS);
    typedef char vofa_tick_check[(pdMS_TO_TICKS(VOFA_PERIOD_MS) > 0) ? 1 : -1];
    (void)sizeof(vofa_tick_check);
#endif
    Usart1Init();
    // 空闲一段时间
    vTaskDelay(COMMUNICATION_TASK_INIT_TIME);
#if VOFA_ENABLE
    last_wake = xTaskGetTickCount();
#endif
    while (1) {
#if VOFA_ENABLE
        // Skip old slots, including late wakeups; never burst-send catch-up frames.
        TickType_t late = xTaskGetTickCount() - last_wake;
        if (late >= period) {
            TickType_t missed = late / period;
            last_wake += missed * period;
            VofaRecordMissedPeriods(missed);
        }
#endif
        Uart2TaskLoop();

        // 系统延时
#if VOFA_ENABLE
        late = xTaskGetTickCount() - last_wake;
        if (late >= period) {
            TickType_t missed = late / period;
            last_wake += missed * period;
            VofaRecordMissedPeriods(missed);
        }
        vTaskDelayUntil(&last_wake, period);
#else
        vTaskDelay(COMMUNICATION_TASK_TIME_MS);
#endif

#if INCLUDE_uxTaskGetStackHighWaterMark
        communication_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
