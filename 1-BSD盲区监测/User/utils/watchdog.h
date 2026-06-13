#ifndef __WATCHDOG_H__
#define __WATCHDOG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化并启动独立看门狗 (IWDG), 超时约 200ms */
void watchdog_init(void);

/* 喂狗，需在主循环中周期性调用 */
void watchdog_feed(void);

#ifdef __cplusplus
}
#endif

#endif /* __WATCHDOG_H__ */
