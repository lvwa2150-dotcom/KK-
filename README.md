# BSD 盲区监测系统

基于 STM32F103C8 + MS60 毫米波雷达的两轮车盲区监测（BSD）系统，提供 LCA（变道辅助）和 RCW（后方碰撞预警）功能，通过 LED 灯光实时提醒驾驶员。

## 硬件平台

| 组件 | 型号 |
|------|------|
| 主控 MCU | STM32F103C8 |
| 毫米波雷达 | MS60（UART 通信） |
| 左侧 LED | PA4 |
| 右侧 LED | PA5 |
| 调试串口 | USART2 (PA2/PA3, 115200) |

## 功能特性

### 雷达数据采集
- 通过 UART DMA 接收 MS60 雷达原始数据，配合环形缓冲区（512 字节）实现高吞吐无阻塞读取
- 状态机流式解析雷达帧协议（帧头 `0x5A`，最大 8 个目标）

### 区域划分
将车辆周围分为三个监测区域：

| 区域 | 横向范围 | 说明 |
|------|----------|------|
| 左侧 LCA | -5000mm ~ -1000mm | 左后方变道辅助 |
| 中间 RCW | -1000mm ~ 1000mm | 正后方碰撞预警 |
| 右侧 LCA | 1000mm ~ 5000mm | 右后方变道辅助 |

### 三级预警等级

| 等级 | 触发条件 | LED 状态 |
|------|----------|----------|
| LEVEL_3 | 距离 < 7m | 常亮 |
| LEVEL_2 | 距离 < 30m | 4Hz 快闪 |
| LEVEL_1 | 距离 < 50m | 1Hz 慢闪 |
| LEVEL_NONE | 距离 >= 50m | 熄灭 |

- RCW（中间区域）触发时左右两侧 LED 同时响应
- 支持目标跟踪与去重，过滤单帧幽灵目标
- 帧间平滑保持，防止预警等级抖动

### 系统架构
采用分时轮询调度器，三个任务协同工作：

| 任务 | 周期 | 功能 |
|------|------|------|
| `task_radar_poll` | 1ms | 更新 DMA 写指针 + 状态机解析雷达帧 |
| `task_warning_process` | 10ms | 预警等级计算 + LED 同步 |
| `task_debug_output` | 1000ms | 调试信息输出（帧率、错误率、目标数据） |

### 安全机制
- 独立看门狗（IWDG），超时约 500ms，任务异常自动复位
- 外部晶振（HSE）起振失败检测，双 LED 快闪报警
- DMA 溢出检测与计数
- 雷达数据超时保护（1 秒无数据则复位预警状态）

## 目录结构

```
├── Library/          # STM32F10x 标准外设库
├── Start/            # CMSIS 启动文件
├── System/           # 系统延时模块
├── User/
│   ├── led/          # LED 驱动（PA4/PA5）
│   ├── radar/        # MS60 雷达协议解析
│   ├── systick/      # SysTick 系统滴答
│   ├── utils/        # 工具模块
│   │   ├── ring_buffer.c   # DMA 环形缓冲区
│   │   ├── scheduler.c     # 分时轮询调度器
│   │   ├── debug_log.c     # 调试日志输出
│   │   └── watchdog.c      # 看门狗管理
│   ├── warning/      # 预警算法（区域判定 + 等级计算）
│   └── main.c        # 主程序入口
└── Project.uvprojx   # Keil MDK 工程文件
```

## 使用说明

### 开发环境
- **IDE**: Keil MDK-ARM V5
- **编译器**: ARMCC V5
- **目标芯片**: STM32F103C8（64KB Flash, 20KB SRAM）
- **仿真器**: ST-Link / J-Link

### 编译与烧录
1. 用 Keil MDK 打开 `1-BSD盲区监测/Project.uvprojx`
2. 确认目标芯片为 STM32F103C8
3. 编译（F7）通过后，连接 ST-Link 烧录（F8）

### 硬件接线

| 功能 | MCU 引脚 | 外设 |
|------|----------|------|
| 雷达 UART RX | PA10 (USART1_RX) | MS60 TX |
| 雷达 UART TX | PA9 (USART1_TX) | MS60 RX |
| 左侧 LED | PA4 | LED |
| 右侧 LED | PA5 | LED |
| 调试串口 TX | PA2 (USART2_TX) | USB-TTL 模块 RX |

### 调试输出
调试串口 (USART2, 115200-8-N-1) 每秒输出一次状态信息：
- 帧总数 / 错误帧数 / 错误率
- DMA 溢出次数
- 各区域当前预警等级（RCW 贡献后 / LCA 原生等级）

若需关闭调试输出，在 `User/utils/debug_log.h` 中注释 `#define DEBUG_LOG_ENABLE`。

### 开机自检
系统上电后自动执行：
1. 外部晶振检测 — 失败则双 LED 快闪报警
2. LED 自检 — 两侧 LED 各闪 3 下（亮 500ms / 灭 500ms）
3. 进入正常监测流程

## 许可证

本项目采用 [MIT License](LICENSE)。
