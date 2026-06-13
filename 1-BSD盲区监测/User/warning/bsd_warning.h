#ifndef __BSD_WARNING_H__
#define __BSD_WARNING_H__

#include <stdint.h>
#include "radar/bsd_ms60.h"

/*
 * 车辆横向区域定义 (总宽度 10500mm，三等分各 3500mm):
 *   左侧LCA: X ∈ [-5250, -1000],  宽度 4250mm
 *   中间RCW: X ∈ [-1000,  1000],  宽度 2000mm
 *   右侧LCA: X ∈ [ 1000,  5250],  宽度 4250mm
 *
 * LED 输出聚合:
 *   BSD_ZONE_LEFT   → 左侧 LED
 *   BSD_ZONE_RIGHT  → 右侧 LED
 *   BSD_ZONE_REAR   → 双侧 LED (RCW 中间区域)
 */
typedef enum {
    BSD_ZONE_LEFT  = 0,
    BSD_ZONE_RIGHT = 1,
    BSD_ZONE_REAR  = 2,
    BSD_ZONE_MAX
} bsd_zone_t;

/*
 * TTC 三级预警等级 (距离 ÷ 靠近速度):
 *   LEVEL_1: TTC < 10s  远端低风险 → LED 1Hz 慢闪
 *   LEVEL_2: TTC <  7s  中风险     → LED 4Hz 快闪
 *   LEVEL_3: TTC <  3s  极近高风险 → LED 常亮
 *
 * 优先级: LEVEL_3 > LEVEL_2 > LEVEL_1 > LEVEL_NONE
 */
typedef enum {
    BSD_LEVEL_NONE = 0,
    BSD_LEVEL_1    = 1,
    BSD_LEVEL_2    = 2,
    BSD_LEVEL_3    = 3,
} bsd_level_t;

/**
 * @brief 初始化预警系统(含LED初始化)
 */
void bsd_warning_init(void);

/**
 * @brief 根据雷达数据更新各区域预警等级
 * @param radar_data 雷达检测数据指针
 * @note  在主循环中调用,建议配合 BSD_MS60_HasNewData() 使用
 */
void bsd_warning_update(const bsd_det_info_t *radar_data);

/**
 * @brief 将预警等级同步到LED显示
 * @note  建议每1ms或在主循环中周期性调用
 */
void bsd_warning_process(void);

/**
 * @brief 获取指定区域的当前预警等级（含 RCW 交叉贡献）
 * @param zone 区域枚举
 * @return 该区域预警等级
 */
bsd_level_t bsd_warning_get_level(bsd_zone_t zone);

/**
 * @brief 获取指定区域的 LCA 原生等级（不含 RCW 交叉贡献，用于区分触发来源）
 * @param zone 区域枚举（BSD_ZONE_REAR 永远返回 BSD_LEVEL_NONE）
 * @return 该区域 LCA 原生预警等级
 */
bsd_level_t bsd_warning_get_lca_level(bsd_zone_t zone);

#endif /* __BSD_WARNING_H__ */
