#include "debug_log.h"
#include "radar/bsd_ms60.h"
#include "warning/bsd_warning.h"
#include "stm32f10x.h"

static void debug_putchar(char c)
{
    uint32_t timeout = 10000;
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
        if (--timeout == 0) {
            return;
        }
    }
    USART_SendData(USART2, (uint8_t)c);
}

static void debug_puts(const char *s)
{
    while (*s) {
        debug_putchar(*s++);
    }
}

static void debug_putu32(uint32_t val)
{
    char buf[12];
    int i = 0;
    if (val == 0) {
        debug_putchar('0');
        return;
    }
    while (val > 0) {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    }
    while (i > 0) {
        debug_putchar(buf[--i]);
    }
}

void debug_log_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    /* 使能 GPIOA 和 AFIO 时钟（APB2） */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    /* 使能 USART2 时钟（APB1） */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* PA2 = USART2_TX，复用推挽 */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 配置 USART2：115200, 8N1，仅发送 */
    USART_InitStruct.USART_BaudRate            = 115200;
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;
    USART_InitStruct.USART_Parity              = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode                = USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStruct);
    USART_Cmd(USART2, ENABLE);
}

void debug_log_print(void)
{
    /* 输出格式: BSD:total,err,overflow,L,R,Rear,L_lca,R_lca\n */
    debug_puts("BSD:");
    debug_putu32(BSD_MS60_GetFrameTotalCnt());
    debug_putchar(',');
    debug_putu32(BSD_MS60_GetFrameErrCnt());
    debug_putchar(',');
    debug_putu32(BSD_MS60_GetOverflowCnt());
    debug_putchar(',');
    debug_putu32(bsd_warning_get_level(BSD_ZONE_LEFT));
    debug_putchar(',');
    debug_putu32(bsd_warning_get_level(BSD_ZONE_RIGHT));
    debug_putchar(',');
    debug_putu32(bsd_warning_get_level(BSD_ZONE_REAR));
    debug_putchar(',');
    debug_putu32(bsd_warning_get_lca_level(BSD_ZONE_LEFT));
    debug_putchar(',');
    debug_putu32(bsd_warning_get_lca_level(BSD_ZONE_RIGHT));
    debug_putchar('\n');
}
