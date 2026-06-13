#include "bsd_ms60.h"
#include "utils/ring_buffer.h"

/* 雷达帧解析状态机 */
typedef struct {
    uint8_t  frame_buf[BSD_MS60_MAX_FRAME_LEN];
    uint8_t  frame_idx;
    uint16_t frame_total_len;
    uint8_t  state;
} ms60_parser_t;

// DMA 环形缓冲区 (物理上由 DMA 直接写入)
static ring_buffer_t s_radar_rb;

// 状态机实例
static ms60_parser_t s_parser;

// 全局雷达数据
bsd_det_info_t g_radar_data;

// 新数据标志位
static volatile uint8_t newDataFlag = 0;

// 帧解析统计（调试排错用）
static uint32_t s_frame_total_cnt = 0;
static uint32_t s_frame_err_cnt   = 0;

// DMA 溢出统计
static uint32_t s_dma_overflow_cnt = 0;

/* 连续错误帧阈值：超过此值主动清空缓冲区强制同步 */
#define FRAME_ERROR_BURST_THRESHOLD  20
static uint8_t s_consecutive_err = 0;

// 内部函数声明
static void parser_reset(ms60_parser_t *p);
static void parser_feed_byte(ms60_parser_t *p, uint8_t data);
static uint8_t parser_parse_frame(uint8_t *frame, uint8_t len);
static uint8_t handle_frame_error(void);

/**
 * @brief 重置状态机
 */
static void parser_reset(ms60_parser_t *p)
{
    p->state = 0;
    p->frame_idx = 0;
    p->frame_total_len = 0;
}

/**
 * @brief 帧错误统一处理：记录错误，连续过多时强制清空缓冲区同步
 * @return 1（始终返回错误）
 */
static uint8_t handle_frame_error(void)
{
    s_frame_err_cnt++;
    s_consecutive_err++;
    if (s_consecutive_err >= FRAME_ERROR_BURST_THRESHOLD) {
        s_consecutive_err = 0;
        rb_discard_all(&s_radar_rb);
        parser_reset(&s_parser);
    }
    return 1;
}

/**
 * @brief 解析单帧数据
 * @return 0: 成功, 1: 失败
 */
static uint8_t parser_parse_frame(uint8_t *frame, uint8_t len)
{
    uint8_t i;
    uint16_t payloadLen;
    uint8_t checksum;
    uint16_t sum;

    s_frame_total_cnt++;

    if (frame[0] != BSD_MS60_FRAME_HEAD1) {
        return handle_frame_error();
    }

    payloadLen = frame[1];
    if (len != (1 + 1 + payloadLen + 1)) {
        return handle_frame_error();
    }

    checksum = frame[1 + 1 + payloadLen];

    sum = 0;
    sum += frame[0];
    sum += frame[1];
    for (i = 0; i < payloadLen; i++) {
        sum += frame[2 + i];
    }

    if ((sum & 0xFF) != checksum) {
        return handle_frame_error();
    }

    uint8_t type = frame[2];
    if (type != BSD_MS60_FRAME_TYPE) {
        return handle_frame_error();
    }

    uint16_t obj_num = frame[3] | (frame[4] << 8);
    if (obj_num > BSD_MS60_TARGET_MAX) {
        obj_num = BSD_MS60_TARGET_MAX;
    }

    /* 校验 obj_num 与 payloadLen 是否一致，防止解析垃圾数据 */
    uint16_t expected_payload = BSD_MS60_PAYLOAD_HEADER_LEN + obj_num * BSD_MS60_TARGET_DATA_LEN;
    if (payloadLen != expected_payload) {
        return handle_frame_error();
    }

    g_radar_data.obj_num = obj_num;

    g_radar_data.reserved = frame[5] | (frame[6] << 8);

    uint8_t dataOffset = 2 + BSD_MS60_PAYLOAD_HEADER_LEN;
    for (i = 0; i < obj_num; i++) {
        uint8_t baseOffset = dataOffset + i * BSD_MS60_TARGET_DATA_LEN;
        uint8_t d0 = frame[baseOffset];
        uint8_t d1 = frame[baseOffset + 1];
        uint8_t d2 = frame[baseOffset + 2];
        uint8_t d3 = frame[baseOffset + 3];

        g_radar_data.targets[i].distance = d0;
        g_radar_data.targets[i].angle    = (int8_t)d1;
        g_radar_data.targets[i].speed    = (int8_t)d2; /* 雷达 velo 负=靠近，与代码语义一致，直接存入 */
        g_radar_data.targets[i].id       = d3;
    }

    s_consecutive_err = 0;
    newDataFlag = 1;
    return 0;
}

/**
 * @brief 状态机处理单个字节
 */
static void parser_feed_byte(ms60_parser_t *p, uint8_t data)
{
    switch (p->state) {
        case 0:
            if (data == BSD_MS60_FRAME_HEAD1) {
                p->frame_buf[0] = data;
                p->frame_idx = 1;
                p->state = 1;
            }
            break;

        case 1:
            p->frame_buf[p->frame_idx++] = data;
            if (p->frame_idx >= 2) {
                uint16_t payloadLen = p->frame_buf[1];
                p->frame_total_len = 1 + 1 + payloadLen + 1;
                if (p->frame_total_len > sizeof(p->frame_buf)) {
                    parser_reset(p);
                } else {
                    p->state = 2;
                }
            }
            break;

        case 2:
            p->frame_buf[p->frame_idx++] = data;
            if (p->frame_idx >= p->frame_total_len) {
                parser_parse_frame(p->frame_buf, p->frame_idx);
                parser_reset(p);
            } else if (p->frame_idx >= sizeof(p->frame_buf)) {
                parser_reset(p);
            }
            break;

        default:
            parser_reset(p);
            break;
    }
}

/**
 * @brief BSD_MS60雷达模块初始化函数
 */
void BSD_MS60_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    DMA_InitTypeDef DMA_InitStruct;

    // 初始化环形缓冲区
    rb_init(&s_radar_rb);
    parser_reset(&s_parser);
    newDataFlag = 0;

    // 使能GPIOA和AFIO时钟（APB2总线）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    // 使能USART1时钟（APB2总线）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    // 使能DMA1时钟（AHB总线）
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // 配置USART1_TX引脚（PA9）为复用推挽输出
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 配置USART1_RX引脚（PA10）为浮空输入
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 配置USART1参数：波特率921600，8位数据位，1位停止位，无校验位
    USART_InitStruct.USART_BaudRate = 921600;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStruct);

    // 配置DMA1_Channel5用于USART1接收 (环形缓冲区直接作为 DMA 目标地址)
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)s_radar_rb.buffer;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStruct.DMA_BufferSize = RB_SIZE;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStruct);

    // 使能DMA通道和USART1的DMA接收请求
    DMA_Cmd(DMA1_Channel5, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    // 使能USART1
    USART_Cmd(USART1, ENABLE);
}

/**
 * @brief 通过USART1发送单个字节
 */
void BSD_MS60_SendByte(uint8_t data)
{
    uint32_t timeout = 10000;
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
        if (--timeout == 0) {
            return;  /* 异常分支：硬件故障时避免死循环 */
        }
    }
    USART_SendData(USART1, data);
}

/**
 * @brief 更新环形缓冲区写指针（根据 DMA 计数器）
 */
void BSD_MS60_PollDMA(void)
{
    rb_update_write_from_dma(&s_radar_rb,
                             DMA_GetCurrDataCounter(DMA1_Channel5),
                             RB_SIZE);
    /* DMA 溢出时重置解析器，丢弃可能残缺的帧 */
    if (s_radar_rb.overflow) {
        s_dma_overflow_cnt++;
        parser_reset(&s_parser);
        BSD_MS60_ClearOverflowFlag();
    }
}

/**
 * @brief 从环形缓冲区批量取字节送入状态机解析
 */
void BSD_MS60_ParseStream(void)
{
    uint8_t data;
    while (rb_read(&s_radar_rb, &data)) {
        parser_feed_byte(&s_parser, data);
    }
}

/**
 * @brief 旧接口兼容层：PollDMA + ParseStream
 */
void BSD_MS60_ProcessDMA(void)
{
    BSD_MS60_PollDMA();
    BSD_MS60_ParseStream();
}

/**
 * @brief 获取雷达检测数据
 */
uint8_t BSD_MS60_GetData(bsd_det_info_t *data)
{
    if (newDataFlag) {
        *data = g_radar_data;
        newDataFlag = 0;
        return 0;
    }
    return 1;
}

/**
 * @brief 检查是否有新数据
 */
uint8_t BSD_MS60_HasNewData(void)
{
    return newDataFlag ? 1 : 0;
}

/**
 * @brief 清除新数据标志位
 */
void BSD_MS60_ClearDataFlag(void)
{
    newDataFlag = 0;
}

/**
 * @brief 获取总解析帧数（含错误帧）
 */
uint32_t BSD_MS60_GetFrameTotalCnt(void)
{
    return s_frame_total_cnt;
}

/**
 * @brief 获取错误帧数
 */
uint32_t BSD_MS60_GetFrameErrCnt(void)
{
    return s_frame_err_cnt;
}

/**
 * @brief 重置帧解析统计
 */
void BSD_MS60_ResetFrameStats(void)
{
    s_frame_total_cnt = 0;
    s_frame_err_cnt   = 0;
}

/**
 * @brief 获取 DMA 溢出标志
 */
uint8_t BSD_MS60_GetOverflowFlag(void)
{
    return s_radar_rb.overflow;
}

/**
 * @brief 清除 DMA 溢出标志
 */
void BSD_MS60_ClearOverflowFlag(void)
{
    s_radar_rb.overflow = 0;
}

/**
 * @brief 获取 DMA 溢出次数（累计）
 */
uint32_t BSD_MS60_GetOverflowCnt(void)
{
    return s_dma_overflow_cnt;
}

/**
 * @brief 清除 DMA 溢出次数
 */
void BSD_MS60_ClearOverflowCnt(void)
{
    s_dma_overflow_cnt = 0;
}
