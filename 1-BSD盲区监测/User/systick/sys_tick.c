#include "sys_tick.h"
#include "led/led.h"

static volatile uint32_t s_sys_tick_ms = 0;

uint32_t get_tick_ms(void)
{
    return s_sys_tick_ms;
}

void sys_tick_inc(void)
{
    s_sys_tick_ms++;
    led_tick_handler();  /* 同步驱动LED闪烁时基 */
}

void sys_tick_init(void)
{
    s_sys_tick_ms = 0;
    /* SystemCoreClock 通常为 72MHz, SysTick_Config 配置 1ms 中断 */
    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1);  /* 配置失败 */
    }

    /* 使能 DWT CYCCNT 用于精准微秒延时 */
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void sys_tick_ms(uint32_t ms)
{
    uint32_t start = get_tick_ms();
    while ((get_tick_ms() - start) < ms) {
        /* 阻塞等待 */
    }
}

void sys_tick_us(uint32_t us)
{
    /* 使用 DWT CYCCNT 实现精准微秒延时 */
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < ticks) {
        __NOP();
    }
}
