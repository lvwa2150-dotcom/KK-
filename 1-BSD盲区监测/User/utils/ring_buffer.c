#include "ring_buffer.h"

void rb_init(ring_buffer_t *rb)
{
    rb->write_idx = 0;
    rb->read_idx  = 0;
    rb->overflow  = 0;
    int i;
    for (i = 0; i < RB_SIZE; i++) {
        rb->buffer[i] = 0;
    }
}

void rb_update_write_from_dma(ring_buffer_t *rb, uint16_t dma_curr_data_counter, uint16_t dma_buffer_size)
{
    uint16_t new_pos = dma_buffer_size - dma_curr_data_counter;
    uint16_t old_pos = rb->write_idx & RB_MASK;

    uint16_t delta;
    if (new_pos >= old_pos) {
        delta = new_pos - old_pos;
    } else {
        /* DMA 已经回绕一圈 */
        delta = (dma_buffer_size - old_pos) + new_pos;
    }

    /* 检测 DMA 是否发生溢出 */
    uint16_t curr_count = rb_count(rb);
    if (curr_count + delta > RB_SIZE) {
        rb->overflow = 1;
    }

    rb->write_idx += delta;
}

int rb_read(ring_buffer_t *rb, uint8_t *out)
{
    if (rb->write_idx == rb->read_idx) {
        return 0;  /* 空 */
    }
    *out = rb->buffer[rb->read_idx & RB_MASK];
    rb->read_idx++;
    return 1;
}

int rb_peek(const ring_buffer_t *rb, uint8_t *out)
{
    if (rb->write_idx == rb->read_idx) {
        return 0;  /* 空 */
    }
    *out = rb->buffer[rb->read_idx & RB_MASK];
    return 1;
}

void rb_clear_overflow(ring_buffer_t *rb)
{
    rb->overflow = 0;
}

void rb_discard_all(ring_buffer_t *rb)
{
    rb->read_idx = rb->write_idx;
}
