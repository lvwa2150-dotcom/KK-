#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#include <stdint.h>
#include <stddef.h>

/* 环形缓冲区大小，必须是 2 的幂次方 */
#define RB_SIZE 512
#define RB_MASK (RB_SIZE - 1)

/* 编译时断言：RB_SIZE 必须是 2 的幂次方 */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
#include <assert.h>
_Static_assert((RB_SIZE & (RB_SIZE - 1)) == 0, "RB_SIZE must be power of 2");
#else
typedef char _rb_size_assert[(RB_SIZE & (RB_SIZE - 1)) == 0 ? 1 : -1];
#endif

typedef struct {
    volatile uint16_t write_idx;  /* 由 DMA 更新 */
    volatile uint16_t read_idx;   /* 由 CPU 消费时更新 */
    uint8_t  buffer[RB_SIZE];
    volatile uint8_t  overflow;   /* DMA 溢出标志，1=曾发生溢出 */
} ring_buffer_t;

/* 初始化 */
void rb_init(ring_buffer_t *rb);

/* 根据 DMA 剩余计数器更新 write_idx (size 为 DMA_BufferSize) */
void rb_update_write_from_dma(ring_buffer_t *rb, uint16_t dma_curr_data_counter, uint16_t dma_buffer_size);

/* 当前缓冲区中可读字节数 */
static inline uint16_t rb_count(const ring_buffer_t *rb)
{
    return (uint16_t)(rb->write_idx - rb->read_idx);
}

/* 读取 1 个字节，返回 1 表示成功，0 表示空 */
int rb_read(ring_buffer_t *rb, uint8_t *out);

/* 偷看 1 个字节但不移动读指针，返回 1 表示成功，0 表示空 */
int rb_peek(const ring_buffer_t *rb, uint8_t *out);

/* 清除 DMA 溢出标志 */
void rb_clear_overflow(ring_buffer_t *rb);

/* 丢弃缓冲区中所有未读数据（read_idx 追上 write_idx） */
void rb_discard_all(ring_buffer_t *rb);

#endif /* __RING_BUFFER_H__ */
