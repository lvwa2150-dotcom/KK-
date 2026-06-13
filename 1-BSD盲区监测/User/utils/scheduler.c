#include "scheduler.h"
#include "systick/sys_tick.h"

static scheduler_task_t s_tasks[SCHEDULER_MAX_TASKS];

int scheduler_register(scheduler_task_fn_t func, uint32_t period_ms)
{
    int i;
    for (i = 0; i < SCHEDULER_MAX_TASKS; i++) {
        if (!s_tasks[i].active) {
            s_tasks[i].func       = func;
            s_tasks[i].period_ms  = period_ms;
            s_tasks[i].last_run_ms = get_tick_ms();
            s_tasks[i].active     = 1;
            return 0;
        }
    }
    return -1;  /* 已满 */
}

void scheduler_run(void)
{
    uint32_t now = get_tick_ms();
    int i;
    for (i = 0; i < SCHEDULER_MAX_TASKS; i++) {
        if (s_tasks[i].active) {
            if ((now - s_tasks[i].last_run_ms) >= s_tasks[i].period_ms) {
                /* 累加周期，避免任务执行耗时导致的调度漂移 */
                s_tasks[i].last_run_ms += s_tasks[i].period_ms;
                /* 若严重滞后，追赶到最近一个周期前 */
                if ((now - s_tasks[i].last_run_ms) >= s_tasks[i].period_ms) {
                    s_tasks[i].last_run_ms = now - s_tasks[i].period_ms;
                }
                s_tasks[i].func();
            }
        }
    }
}
