#ifndef __LED_H__
#define __LED_H__

#include "stm32f10x.h"

/* LED引脚定义 */
#define LED_LEFT_PIN    GPIO_Pin_4
#define LED_LEFT_PORT   GPIOA
#define LED_RIGHT_PIN   GPIO_Pin_5
#define LED_RIGHT_PORT  GPIOA

/* LED侧边定义 */
typedef enum {
    LED_SIDE_LEFT  = 0,  /* 左侧LED: PA4 */
    LED_SIDE_RIGHT = 1,  /* 右侧LED: PA5 */
    LED_SIDE_MAX
} led_side_t;

/* LED状态定义 */
typedef enum {
    LED_STATE_OFF        = 0,  /* LEVEL0: 熄灭 */
    LED_STATE_SLOW_BLINK = 1,  /* LEVEL1: 慢闪 1Hz */
    LED_STATE_FAST_BLINK = 2,  /* LEVEL2: 快闪 4Hz */
    LED_STATE_ON         = 3,  /* LEVEL3: 常亮 */
} led_state_t;

/* 初始化LED GPIO */
void led_init(void);

/* 设置指定侧LED状态 */
void led_set_state(led_side_t side, led_state_t state);

/* 开机自检：两侧LED闪3下，亮500ms/灭500ms */
void led_self_test(void);

/* 1ms定时处理,需在SysTick或1ms定时中断中调用 */
void led_tick_handler(void);


#endif /* __LED_H__ */
