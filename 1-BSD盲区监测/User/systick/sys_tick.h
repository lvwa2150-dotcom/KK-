#ifndef __SYS_TICK_H__
#define __SYS_TICK_H__

#include "stm32f10x.h"

/* 系统定时器初始化(配置为1ms中断) */
void sys_tick_init(void);

/* 毫秒延时(阻塞) */
void sys_tick_ms(uint32_t ms);

/* 微秒延时(阻塞,粗略) */
void sys_tick_us(uint32_t us);

/* 获取系统运行毫秒数 */
uint32_t get_tick_ms(void);

/* 系统滴答递增(在SysTick中断中调用) */
void sys_tick_inc(void);

#endif /* __SYS_TICK_H__ */
