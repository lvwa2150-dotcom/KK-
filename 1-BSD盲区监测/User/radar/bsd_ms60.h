#ifndef __BSD_MS60_H
#define __BSD_MS60_H
#include "stm32f10x.h"

#define BSD_MS60_FRAME_HEAD1        0x5A    // 帧头第1字节
#define BSD_MS60_TARGET_MAX         8       // 最大目标数
#define BSD_MS60_TARGET_DATA_LEN    4       // 每个目标数据长度 (距离1B + 角度1B + 速度1B + ID1B)
#define BSD_MS60_FRAME_TYPE         0x07    // 帧类型
#define BSD_MS60_PAYLOAD_HEADER_LEN 5       // Payload中目标数据前的字节数 (TYPE 1B + obj_num 2B + reserved 2B)
// 最大帧长度 = HEAD(1B) + LEN(1B) + PAYLOAD + CHECK(1B)
#define BSD_MS60_MAX_FRAME_LEN      (1 + 1 + BSD_MS60_PAYLOAD_HEADER_LEN + BSD_MS60_TARGET_MAX * BSD_MS60_TARGET_DATA_LEN + 1)

// 单个目标数据结构
typedef struct {
    uint8_t distance;
    int8_t angle;
    int8_t speed;
    uint8_t id;
} bsd_target_info_t;

// 检测数据结构
typedef struct {
    uint16_t obj_num;
    uint16_t reserved;
    bsd_target_info_t targets[BSD_MS60_TARGET_MAX];
} bsd_det_info_t;

// 全局雷达数据
extern bsd_det_info_t g_radar_data;

// 函数声明
void BSD_MS60_Init(void);                       // 初始化BSD_MS60
void BSD_MS60_SendByte(uint8_t data);           // 发送字节

// 旧接口兼容层：PollDMA + ParseStream
void BSD_MS60_ProcessDMA(void);

// 新接口：DMA 环形缓冲区更新 + 状态机流式解析
void BSD_MS60_PollDMA(void);
void BSD_MS60_ParseStream(void);

uint8_t BSD_MS60_GetData(bsd_det_info_t *data); // 获取检测数据
uint8_t BSD_MS60_HasNewData(void);              // 检查是否有新数据
void BSD_MS60_ClearDataFlag(void);              // 清除数据标志位

// 帧解析统计（调试排错用）
uint32_t BSD_MS60_GetFrameTotalCnt(void);       // 获取总帧数
uint32_t BSD_MS60_GetFrameErrCnt(void);         // 获取错误帧数
void     BSD_MS60_ResetFrameStats(void);        // 重置统计

uint8_t  BSD_MS60_GetOverflowFlag(void);        // 获取 DMA 溢出标志
void     BSD_MS60_ClearOverflowFlag(void);      // 清除 DMA 溢出标志

uint32_t BSD_MS60_GetOverflowCnt(void);         // 获取 DMA 溢出次数（累计）
void     BSD_MS60_ClearOverflowCnt(void);       // 清除 DMA 溢出次数

#endif /* __BSD_MS60_H */
