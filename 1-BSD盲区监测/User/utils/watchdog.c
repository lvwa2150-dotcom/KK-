#include "watchdog.h"
#include "stm32f10x.h"

/*
 * LSI 典型频率 40kHz (范围约 30~60kHz)
 * 超时时间 = (Prescaler * Reload) / LSI_Freq
 * 系统最长任务周期为 10ms，取 50 倍裕量 => 约 500ms
 * Prescaler = 16, Reload = 1250 => 500ms @ 40kHz
 */
void watchdog_init(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_16);
    IWDG_SetReload(1250);
    IWDG_ReloadCounter();
    IWDG_Enable();
}

void watchdog_feed(void)
{
    IWDG_ReloadCounter();
}
