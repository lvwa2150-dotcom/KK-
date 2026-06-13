#include "main.h"
#include "systick/sys_tick.h"
#include "warning/bsd_warning.h"
#include "led/led.h"
#include "radar/bsd_ms60.h"
#include "utils/scheduler.h"
#include "utils/watchdog.h"
#include "utils/debug_log.h"

/* 雷达数据轮询任务：1ms 执行一次 */
static void task_radar_poll(void)
{
    bsd_det_info_t radar_data;

    // 更新 DMA 写指针 + 状态机解析所有新字节
    BSD_MS60_PollDMA();
    BSD_MS60_ParseStream();

    // 获取最新完整帧数据，有则更新预警
    if (BSD_MS60_GetData(&radar_data) == 0) {
        bsd_warning_update(&radar_data);
    }
}

/* 预警处理任务：10ms 执行一次 */
static void task_warning_process(void)
{
    bsd_warning_process();
}

/* 调试输出任务：1000ms 执行一次 */
static void task_debug_output(void)
{
    debug_log_print();
}

int main(void)
{
    // 系统初始化
    sys_tick_init();

    // 雷达模块初始化
    BSD_MS60_Init();

    /* 检查外部晶振(HSE)是否起振成功。
     * 若HSE失败，系统会退回到HSI 8MHz，但代码仍按72MHz计算，
     * 导致波特率严重偏差、完全收不到数据。此时LED快闪报警。 */
    if ((RCC->CR & RCC_CR_HSERDY) == RESET) {
        led_init();
        led_set_state(LED_SIDE_LEFT,  LED_STATE_FAST_BLINK);
        led_set_state(LED_SIDE_RIGHT, LED_STATE_FAST_BLINK);
        while (1) {
            __WFI();  /* SysTick ISR 驱动 LED 闪烁，无需 busy-wait */
        }
    }

    // 预警系统初始化
    bsd_warning_init();

    // 开机自检：两侧LED闪3下
    led_self_test();

    // 调试串口初始化 (USART2 PA2/PA3, 115200)
    debug_log_init();

    // 清除雷达统计和 DMA 溢出标志（调试排错基准清零）
    BSD_MS60_ResetFrameStats();

    // 启动看门狗 (超时约 500ms，基于最长任务周期 10ms 留足裕量)
    watchdog_init();

    // 注册分时轮询任务
    if (scheduler_register(task_radar_poll,      1)  != 0 ||   // 1ms  雷达数据轮询
        scheduler_register(task_warning_process, 10) != 0 ||   // 10ms 预警与LED同步
        scheduler_register(task_debug_output,    1000) != 0) { // 1000ms 调试状态输出
        // 异常分支：任务注册失败，停止喂狗等待复位
        while (1);
    }

    while (1)
    {
        scheduler_run();
        watchdog_feed();  // 仅在主循环喂狗，中断内不喂
        __WFI();          // 无任务时进入睡眠，等 SysTick/DMA 中断唤醒
    }
}
