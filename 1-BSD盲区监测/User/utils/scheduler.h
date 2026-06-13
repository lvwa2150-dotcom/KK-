#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include <stdint.h>

/* 最大任务数 */
#define SCHEDULER_MAX_TASKS 8

typedef void (*scheduler_task_fn_t)(void);

typedef struct {
    scheduler_task_fn_t func;
    uint32_t            period_ms;
    uint32_t            last_run_ms;
    uint8_t             active;
} scheduler_task_t;

/* 注册一个周期性任务，返回 0 成功，-1 失败（已满） */
int scheduler_register(scheduler_task_fn_t func, uint32_t period_ms);

/* 在主循环中调用，轮询并执行到期任务 */
void scheduler_run(void);

#endif /* __SCHEDULER_H__ */
