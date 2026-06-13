#include "led.h"
#include "systick/sys_tick.h"

/* 闪烁半周期定义 (ms),  toggle周期 = 2 * HALF_PERIOD */
#define LED_SLOW_HALF_PERIOD  500u  /* 1Hz: 500ms亮/500ms灭 */
#define LED_FAST_HALF_PERIOD  125u  /* 4Hz: 125ms亮/125ms灭 */

typedef struct {
    led_state_t state;      /* 当前状态 */
    uint8_t     pin_val;    /* 当前引脚电平: 1=点亮(高电平有效), 0=熄灭 */
    uint16_t    timer;      /* 闪烁计时器 */
} led_ctrl_t;

static volatile led_ctrl_t s_leds[LED_SIDE_MAX];

/**
 * @brief 设置LED引脚电平
 * @param side LED侧边
 * @param on   1=点亮, 0=熄灭
 */
static void led_gpio_set(led_side_t side, uint8_t on)
{
    GPIO_TypeDef *port;
    uint16_t pin;

    if (side >= LED_SIDE_MAX) {
        return;  /* 越界保护 */
    }

    if (side == LED_SIDE_LEFT) {
        port = LED_LEFT_PORT;
        pin  = LED_LEFT_PIN;
    } else {
        port = LED_RIGHT_PORT;
        pin  = LED_RIGHT_PIN;
    }

    if (on) {
        GPIO_SetBits(port, pin);  /* 高电平点亮 */
    } else {
        GPIO_ResetBits(port, pin);    /* 低电平熄灭 */
    }
}

void led_init(void)
{
    GPIO_InitTypeDef gpio;

    /* 使能GPIOA时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* 初始化PA4、PA5 */
    gpio.GPIO_Pin   = LED_LEFT_PIN | LED_RIGHT_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_LEFT_PORT, &gpio);

    /* 默认关闭 */
    int i;
    for (i = 0; i < LED_SIDE_MAX; i++) {
        s_leds[i].state   = LED_STATE_OFF;
        s_leds[i].pin_val = 0;  /* 熄灭 */
        s_leds[i].timer   = 0;
        led_gpio_set((led_side_t)i, 0);
    }
}

/**
 * @brief 开机自检：两侧LED闪3下，亮500ms/灭500ms
 */
void led_self_test(void)
{
    int i;
    for (i = 0; i < 3; i++) {
        led_gpio_set(LED_SIDE_LEFT,  1);
        led_gpio_set(LED_SIDE_RIGHT, 1);
        sys_tick_ms(500);
        led_gpio_set(LED_SIDE_LEFT,  0);
        led_gpio_set(LED_SIDE_RIGHT, 0);
        sys_tick_ms(500);
    }
}

void led_set_state(led_side_t side, led_state_t state)
{
    if (side >= LED_SIDE_MAX) {
        return;
    }

    /* 状态未变时不重置 timer，否则闪烁会失效 */
    if (s_leds[side].state == state) {
        return;
    }

    s_leds[side].state = state;
    s_leds[side].timer = 0;

    switch (state) {
        case LED_STATE_ON:
            s_leds[side].pin_val = 1;  /* 常亮 */
            break;
        case LED_STATE_SLOW_BLINK:
        case LED_STATE_FAST_BLINK:
            s_leds[side].pin_val = 0;  /* 从灭开始,进入闪烁 */
            break;
        default: /* OFF 及非法状态 */
            s_leds[side].pin_val = 0;  /* 熄灭 */
            break;
    }
    led_gpio_set(side, s_leds[side].pin_val);
}

void led_tick_handler(void)
{
    int i;
    for (i = 0; i < LED_SIDE_MAX; i++) {
        uint16_t period = 0;

        switch (s_leds[i].state) {
            case LED_STATE_SLOW_BLINK:
                period = LED_SLOW_HALF_PERIOD;
                break;
            case LED_STATE_FAST_BLINK:
                period = LED_FAST_HALF_PERIOD;
                break;
            default:
                continue;  /* OFF或ON不需要计时处理 */
        }

        s_leds[i].timer++;
        if (s_leds[i].timer >= period) {
            s_leds[i].timer = 0;
            s_leds[i].pin_val = !s_leds[i].pin_val;
            led_gpio_set((led_side_t)i, s_leds[i].pin_val);
        }
    }
}
