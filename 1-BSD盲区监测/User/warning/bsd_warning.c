#include "bsd_warning.h"
#include "led/led.h"
#include "systick/sys_tick.h"
#include <stddef.h>

#define BSD_DIST_SCALE_FACTOR   (1000)

/*
 * 车辆横向区域定义 (总宽度 10000mm，RCW 宽 2000mm，LCA 左右各 4000mm):
 *   左侧LCA: X ∈ [-5000, -1000]
 *   中间RCW: X ∈ [-1000,  1000]
 *   右侧LCA: X ∈ [ 1000,  5000]
 */
#define RCW_HALF_WIDTH_MM       (1000)      /* RCW 半宽 1.0m，覆盖本车全宽 2m */
#define LCA_OUTER_EDGE_MM       (5000)

/*
 * 纯距离三级预警:
 *   dist <  7m → LEVEL_3 (极近, 常亮)
 *   dist < 30m → LEVEL_2 (中风险, 快闪)
 *   dist < 50m → LEVEL_1 (远端低风险, 慢闪)
 *   dist ≥ 50m → LEVEL_NONE
 */
#define BSD_DIST_LEVEL_3_MM     (7000)
#define BSD_DIST_LEVEL_2_MM     (30000)
#define BSD_DIST_LEVEL_1_MM     (50000)

/* 死区生效最小 Y 距离 */
#define BSD_DEAD_ZONE_Y_MIN_MM  (3000)

#define BSD_ANGLE_MAX           (40)

/* ±3° 角度中心死区: 临界地带只走 RCW 路由，不进 LCA 左右分流，消除中线附近左右抖动 */
#define BSD_ANGLE_DEAD_ZONE     (3)

/*
 * 死区 X 位置兜底: 死区仅对 |x| < RCW_HALF + X_MARGIN 的目标生效，
 * 防止远距离小角度目标被死区吸入 RCW。
 * RCW=2m 非机动车道配置，覆盖到 30m/3° (x≈1570mm)。
 */
#define BSD_DEAD_ZONE_X_MARGIN  (600)

/* 帧间平滑保持帧数：按等级峰值缩放，等级越高下降越慢 */
#define LEVEL_HOLD_FRAMES_L3    (6)
#define LEVEL_HOLD_FRAMES_L2    (4)
#define LEVEL_HOLD_FRAMES_L1    (2)

/* 雷达数据超时时间 (毫秒) */
#define BSD_DATA_TIMEOUT_MS     (1000)

/* sin/cos 查表 (Q15 定点), 索引 = angle + 40, 范围：-40° ~ +40° */
static const int16_t SIN_Q15[81] = {
    -21063, -20622, -20174, -19720, -19261, -18795, -18324, -17847, -17364, -16877,
    -16384, -15886, -15384, -14876, -14365, -13848, -13328, -12803, -12275, -11743,
    -11207, -10668, -10126, -9580, -9032, -8481, -7927, -7371, -6813, -6252,
    -5690, -5126, -4560, -3993, -3425, -2856, -2286, -1715, -1144, -572,
    0, 572, 1144, 1715, 2286, 2856, 3425, 3993, 4560, 5126,
    5690, 6252, 6813, 7371, 7927, 8481, 9032, 9580, 10126, 10668,
    11207, 11743, 12275, 12803, 13328, 13848, 14365, 14876, 15384, 15886,
    16384, 16877, 17364, 17847, 18324, 18795, 19261, 19720, 20174, 20622,
    21063
};

static const int16_t COS_Q15[81] = {
    25102, 25466, 25822, 26170, 26510, 26842, 27166, 27482, 27789, 28088,
    28378, 28660, 28932, 29197, 29452, 29698, 29935, 30163, 30382, 30592,
    30792, 30983, 31164, 31336, 31499, 31651, 31795, 31928, 32052, 32166,
    32270, 32365, 32449, 32524, 32588, 32643, 32688, 32723, 32748, 32763,
    32767, 32763, 32748, 32723, 32688, 32643, 32588, 32524, 32449, 32365,
    32270, 32166, 32052, 31928, 31795, 31651, 31499, 31336, 31164, 30983,
    30792, 30592, 30382, 30163, 29935, 29698, 29452, 29197, 28932, 28660,
    28378, 28088, 27789, 27482, 27166, 26842, 26510, 26170, 25822, 25466,
    25102
};

/* 各区域当前预警等级（含 RCW 交叉贡献） */
static bsd_level_t g_zone_level[BSD_ZONE_MAX];

/* 各区域 LCA 原生等级（不含 RCW 交叉贡献，用于区分触发来源） */
static bsd_level_t g_zone_level_lca[BSD_ZONE_MAX];

/* 上一帧各区域预警等级（用于帧间平滑） */
static bsd_level_t g_zone_level_prev[BSD_ZONE_MAX];

/* 等级保持倒计时 */
static uint8_t g_zone_level_hold[BSD_ZONE_MAX];

/* 最后一次收到雷达数据的时间戳 */
static uint32_t g_last_data_time = 0;

/* 目标跟踪：过滤单帧幽灵目标 */
#define TARGET_MAX_TRACK   16
#define TARGET_STABLE_AGE  2

typedef struct {
    uint8_t id;
    uint8_t age;
    uint8_t lost;
} target_track_t;

static target_track_t s_target_track[TARGET_MAX_TRACK];

static void target_track_init(void)
{
    uint8_t i;
    for (i = 0; i < TARGET_MAX_TRACK; i++) {
        s_target_track[i].id         = 0;
        s_target_track[i].age        = 0;
        s_target_track[i].lost       = 0;
    }
}

static uint16_t target_track_update(const bsd_det_info_t *radar_data)
{
    uint8_t i, j;
    uint16_t matched = 0;
    uint16_t stable_mask = 0;
    uint16_t obj_num = radar_data->obj_num;

    if (obj_num > BSD_MS60_TARGET_MAX) {
        obj_num = BSD_MS60_TARGET_MAX;
    }

    for (i = 0; i < obj_num; i++) {
        uint8_t id = radar_data->targets[i].id;
        uint8_t found = 0;
        for (j = 0; j < TARGET_MAX_TRACK; j++) {
            if (s_target_track[j].age > 0 && s_target_track[j].id == id) {
                s_target_track[j].age++;
                if (s_target_track[j].age > 250) {
                    s_target_track[j].age = 250;
                }
                s_target_track[j].lost = 0;
                matched |= ((uint16_t)1 << j);
                found = 1;
                if (s_target_track[j].age >= TARGET_STABLE_AGE) {
                    stable_mask |= ((uint16_t)1 << i);
                }
                break;
            }
        }
        if (!found) {
            for (j = 0; j < TARGET_MAX_TRACK; j++) {
                if (s_target_track[j].age == 0) {
                    s_target_track[j].id         = id;
                    s_target_track[j].age        = 1;
                    s_target_track[j].lost       = 0;
                    matched |= ((uint16_t)1 << j);
                    break;
                }
            }
        }
    }

    for (j = 0; j < TARGET_MAX_TRACK; j++) {
        if (s_target_track[j].age > 0 && !(matched & ((uint16_t)1 << j))) {
            s_target_track[j].lost++;
            if (s_target_track[j].lost >= 3) {
                s_target_track[j].age = 0;
            }
        }
    }

    return stable_mask;
}

static uint8_t bsd_level_hold_frames(bsd_level_t level)
{
    switch (level) {
        case BSD_LEVEL_3: return LEVEL_HOLD_FRAMES_L3;
        case BSD_LEVEL_2: return LEVEL_HOLD_FRAMES_L2;
        default:          return LEVEL_HOLD_FRAMES_L1;
    }
}

static bsd_level_t bsd_level_max(bsd_level_t a, bsd_level_t b)
{
    return (a > b) ? a : b;
}

/*
 * 纯距离三级预警:
 *   dist <  7m → LEVEL_3 (极近, 常亮)
 *   dist < 30m → LEVEL_2 (中风险, 快闪)
 *   dist < 50m → LEVEL_1 (远端低风险, 慢闪)
 *   dist ≥ 50m → LEVEL_NONE
 */
static bsd_level_t bsd_get_level_by_dist(int32_t dist_mm)
{
    if (dist_mm < BSD_DIST_LEVEL_3_MM) {
        return BSD_LEVEL_3;
    }
    if (dist_mm < BSD_DIST_LEVEL_2_MM) {
        return BSD_LEVEL_2;
    }
    if (dist_mm < BSD_DIST_LEVEL_1_MM) {
        return BSD_LEVEL_1;
    }
    return BSD_LEVEL_NONE;
}

/**
 * BSD 盲区预警系统初始化
 * 作用：上电后调用一次，把所有内部状态清零，然后初始化目标跟踪和 LED 硬件。
 * 通俗理解：就像开车前把所有仪表归零，确保系统从干净的状态开始工作。
 */
void bsd_warning_init(void)
{
    int i;
    /* 遍历左侧/右侧/后方三个预警区域（BSD_ZONE_LEFT / BSD_ZONE_RIGHT / BSD_ZONE_REAR） */
    for (i = 0; i < BSD_ZONE_MAX; i++) {
        g_zone_level[i]      = BSD_LEVEL_NONE;  /* 当前综合预警等级（含 RCW 交叉贡献） */
        g_zone_level_lca[i]  = BSD_LEVEL_NONE;  /* 纯 LCA 变道辅助等级（不含 RCW 贡献） */
        g_zone_level_prev[i] = BSD_LEVEL_NONE;  /* 上一帧的等级，用于帧间平滑防抖 */
        g_zone_level_hold[i] = 0;               /* 等级保持计数器（变化太快时不立刻降级） */
    }
    target_track_init();  /* 初始化目标轨迹跟踪（卡尔曼滤波相关） */
    led_init();           /* 初始化 LED 灯硬件（GPIO 配置） */
}

/**
 * BSD 盲区预警核心更新函数
 * 每收到一帧雷达数据就调用一次，处理流程：
 *   1. 数据校验（空指针、目标数上限）
 *   2. 本轮等级清零，准备重新计算
 *   3. 逐目标：过滤 → 极坐标转直角坐标 → 判断区域 → 更新等级
 *   4. 帧间平滑防抖（等级只升不降，下降需要延时）
 */
void bsd_warning_update(const bsd_det_info_t *radar_data)
{
    uint16_t obj_num;
    int i;

    /* 空指针保护：没有雷达数据直接返回 */
    if (radar_data == NULL) {
        return;
    }

    /* 每轮重新计算前，先把所有区域的等级归零 */
    for (i = 0; i < BSD_ZONE_MAX; i++) {
        g_zone_level[i]     = BSD_LEVEL_NONE;
        g_zone_level_lca[i] = BSD_LEVEL_NONE;
    }

    /* 记录本次数据接收时间，用于超时检测（雷达掉线判断） */
    g_last_data_time = get_tick_ms();

    /* 读取目标数量，防止溢出（雷达协议最大支持 32 个目标） */
    obj_num = radar_data->obj_num;
    if (obj_num > BSD_MS60_TARGET_MAX) {
        obj_num = BSD_MS60_TARGET_MAX;
    }

    /* 目标跟踪滤波：卡尔曼/轨迹关联，返回稳定目标的位掩码 */
    uint16_t stable_mask = target_track_update(radar_data);

    /* ---- 逐目标遍历：过滤 + 区域判定 ---- */
    for (i = 0; i < (int)obj_num; i++) {
        int32_t dist_mm = (int32_t)radar_data->targets[i].distance * BSD_DIST_SCALE_FACTOR;
        int8_t  angle   = radar_data->targets[i].angle;
        int8_t  speed   = radar_data->targets[i].speed;

        /* 过滤条件1：只关心接近的目标（speed < 0 表示相对靠近本车） */
        if (speed >= 0) {
            continue;
        }

        /* 过滤条件2：角度超出 ±40° 范围的目标不处理（侧后视场角限制） */
        if (angle < -BSD_ANGLE_MAX || angle > BSD_ANGLE_MAX) {
            continue;
        }

        /* 过滤条件3：距离为 0 的无效目标跳过 */
        if (dist_mm == 0) {
            continue;
        }

        /* 极坐标 → 直角坐标转换：
         *   sin_val / cos_val 是 Q15 定点格式（32768 = 1.0）
         *   右移 15 位完成比例缩放，得到实际的 x（横向）/ y（纵向）毫米坐标
         */
        int idx = (int)angle + 40;
        int16_t sin_val = SIN_Q15[idx];
        int16_t cos_val = COS_Q15[idx];

        int32_t x_mm     = (dist_mm * (int32_t)sin_val) >> 15;
        int32_t y_mm     = (dist_mm * (int32_t)cos_val) >> 15;
        /* 位运算取绝对值：mask = x<0 时为全 1(-1)，否则为 0 */
        int32_t mask     = x_mm >> 31;
        int32_t abs_x_mm = (x_mm ^ mask) - mask;

        /* 过滤条件4：y <= 0 说明目标在后方（雷达数据异常），跳过 */
        if (y_mm <= 0) {
            continue;
        }

        /* 过滤条件5：目标轨迹不稳定的跳过（即目标跟踪滤波器未确认） */
        if (!(stable_mask & ((uint16_t)1 << i))) {
            continue;
        }

        /* 按距离查表得到原始预警等级（<7m→L3, <30m→L2, <50m→L1） */
        bsd_level_t level = bsd_get_level_by_dist(dist_mm);
        if (level == BSD_LEVEL_NONE) {
            continue;
        }

        /*
         * 横向区域路由:
         *   |x| < 1000 → RCW 中间区域 (BSD_ZONE_REAR)
         *   1000 ≤ |x| < 5000 且 x < 0 → 左侧 LCA
         *   1000 ≤ |x| < 5000 且 x > 0 → 右侧 LCA
         *
         * 角度死区: |angle| ≤ 3° 且 y ≥ 3000mm 且 X 在中线附近 → 强制 RCW
         */
        uint8_t via_dead_zone = (y_mm >= BSD_DEAD_ZONE_Y_MIN_MM
                              && angle >= -BSD_ANGLE_DEAD_ZONE && angle <= BSD_ANGLE_DEAD_ZONE
                              && abs_x_mm < RCW_HALF_WIDTH_MM + BSD_DEAD_ZONE_X_MARGIN);
        uint8_t in_rcw = (abs_x_mm < RCW_HALF_WIDTH_MM) || via_dead_zone;

        /* 根据目标所在区域，取该区域所有目标中的最高等级（危险程度取最严重的） */
        if (in_rcw) {
            g_zone_level[BSD_ZONE_REAR] = bsd_level_max(g_zone_level[BSD_ZONE_REAR], level);
        } else if (abs_x_mm < LCA_OUTER_EDGE_MM) {
            if (x_mm < 0) {
                g_zone_level[BSD_ZONE_LEFT] = bsd_level_max(g_zone_level[BSD_ZONE_LEFT], level);
                g_zone_level_lca[BSD_ZONE_LEFT] = bsd_level_max(g_zone_level_lca[BSD_ZONE_LEFT], level);
            } else {
                g_zone_level[BSD_ZONE_RIGHT] = bsd_level_max(g_zone_level[BSD_ZONE_RIGHT], level);
                g_zone_level_lca[BSD_ZONE_RIGHT] = bsd_level_max(g_zone_level_lca[BSD_ZONE_RIGHT], level);
            }
        }
    }

    /* ---- 帧间平滑防抖：等级只升不降，下降需保持 N 帧后才生效 ---- */
    for (i = 0; i < BSD_ZONE_MAX; i++) {
        if (g_zone_level[i] >= g_zone_level_prev[i]) {
            /* 等级升高（或不变）：立即生效，重置保持计数器 */
            g_zone_level_prev[i] = g_zone_level[i];
            g_zone_level_hold[i] = bsd_level_hold_frames(g_zone_level[i]);
        } else {
            /* 等级下降：不立刻降，保持旧等级撑完 hold 帧数再降 */
            if (g_zone_level_hold[i] > 0) {
                g_zone_level_hold[i]--;
                g_zone_level[i] = g_zone_level_prev[i];  /* 维持旧等级 */
            } else {
                g_zone_level_prev[i] = g_zone_level[i];  /* 帧数耗尽，正式降级 */
            }
        }
    }
}

/**
 * 获取指定区域的综合预警等级（含 RCW 交叉贡献）
 * @return: 该区域当前的预警等级
 */
bsd_level_t bsd_warning_get_level(bsd_zone_t zone)
{
    if (zone >= BSD_ZONE_MAX) {
        return BSD_LEVEL_NONE;
    }
    return g_zone_level[zone];
}

/**
 * 获取指定区域的纯 LCA 变道辅助等级（不含 RCW 交叉贡献）
 * 和上面那个函数的区别：这个值不受 RCW（正后方目标）的影响，
 * 只看两侧 LCA 区域的原始等级，用于判断 LED 是双边亮还是单边亮。
 */
bsd_level_t bsd_warning_get_lca_level(bsd_zone_t zone)
{
    if (zone >= BSD_ZONE_MAX) {
        return BSD_LEVEL_NONE;
    }
    return g_zone_level_lca[zone];
}

/**
 * 预警等级 → LED 灯状态转换
 *   L1（远端）→ 慢闪   L2（中距）→ 快闪   L3（极近）→ 常亮   无 → 灭
 */
static led_state_t level_to_led_state(bsd_level_t level)
{
    switch (level) {
        case BSD_LEVEL_1:    return LED_STATE_SLOW_BLINK;
        case BSD_LEVEL_2:    return LED_STATE_FAST_BLINK;
        case BSD_LEVEL_3:    return LED_STATE_ON;
        case BSD_LEVEL_NONE:
        default:             return LED_STATE_OFF;
    }
}

void bsd_warning_process(void)
{
    if (get_tick_ms() - g_last_data_time > BSD_DATA_TIMEOUT_MS) {
        int i;
        for (i = 0; i < BSD_ZONE_MAX; i++) {
            g_zone_level[i]      = BSD_LEVEL_NONE;
            g_zone_level_lca[i]  = BSD_LEVEL_NONE;
            g_zone_level_prev[i] = BSD_LEVEL_NONE;
            g_zone_level_hold[i] = 0;
        }

        led_set_state(LED_SIDE_LEFT,  LED_STATE_OFF);
        led_set_state(LED_SIDE_RIGHT, LED_STATE_OFF);
    } else {
        /*
         * LED 输出聚合:
         *   左侧 LED = max(BSD_ZONE_LEFT, BSD_ZONE_REAR)
         *   右侧 LED = max(BSD_ZONE_RIGHT, BSD_ZONE_REAR)
         *
         * RCW 中间区域 → BSD_ZONE_REAR → 双侧 LED 同时显示
         * LCA 侧方区域 → 仅对应侧 LED 显示
         */
        bsd_level_t left_level  = bsd_level_max(g_zone_level[BSD_ZONE_LEFT],  g_zone_level[BSD_ZONE_REAR]);
        bsd_level_t right_level = bsd_level_max(g_zone_level[BSD_ZONE_RIGHT], g_zone_level[BSD_ZONE_REAR]);

        led_set_state(LED_SIDE_LEFT,  level_to_led_state(left_level));
        led_set_state(LED_SIDE_RIGHT, level_to_led_state(right_level));
    }
}
