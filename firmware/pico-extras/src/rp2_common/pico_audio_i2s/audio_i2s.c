/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>

#include "pico/audio_i2s.h"
#include "audio_i2s.pio.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

#define audio_pio __CONCAT(pio, PICO_AUDIO_I2S_PIO)
#define GPIO_FUNC_PIOx __CONCAT(GPIO_FUNC_PIO, PICO_AUDIO_I2S_PIO)
#define DREQ_PIOx_TX0 __CONCAT(__CONCAT(DREQ_PIO, PICO_AUDIO_I2S_PIO), _TX0)

audio_i2s_state_t i2s_shared_state;

static audio_format_t pio_i2s_consumer_format;
static audio_buffer_format_t pio_i2s_consumer_buffer_format = {
    .format = &pio_i2s_consumer_format,
};

static audio_buffer_t silence_buffer;
static audio_buffer_pool_t *audio_i2s_consumer;

static void __isr __time_critical_func(audio_i2s_dma_irq_handler)();

const audio_format_t *audio_i2s_setup(const audio_format_t *intended_audio_format,
                                      const audio_i2s_config_t *config) {
    uint func = GPIO_FUNC_PIOx;
    gpio_set_function(config->data_pin, func);
    gpio_set_function(config->clock_pin_base, func);
    gpio_set_function(config->clock_pin_base + 1, func);

    uint8_t sm = i2s_shared_state.pio_sm = config->pio_sm;
    pio_sm_claim(audio_pio, sm);

    uint offset = pio_add_program(audio_pio, &audio_i2s_program);
    audio_i2s_program_init(audio_pio, sm, offset, config->data_pin, config->clock_pin_base);

    silence_buffer.buffer = pico_buffer_alloc(PICO_AUDIO_I2S_SILENCE_BUFFER_SAMPLE_LENGTH * 8);
    memset(silence_buffer.buffer->bytes, 0, PICO_AUDIO_I2S_SILENCE_BUFFER_SAMPLE_LENGTH * 8);
    silence_buffer.sample_count = PICO_AUDIO_I2S_SILENCE_BUFFER_SAMPLE_LENGTH;
    silence_buffer.format = &pio_i2s_consumer_buffer_format;

    __mem_fence_release();
    uint8_t dma_channel = config->dma_channel;
    dma_channel_claim(dma_channel);

    i2s_shared_state.dma_channel = dma_channel;
    i2s_shared_state.words_consumed = 0;
    i2s_shared_state.current_transfer_words = 0;

    dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);
    channel_config_set_dreq(&dma_config, DREQ_PIOx_TX0 + sm);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);

#if PICO_RP2350
    channel_config_set_high_priority(&dma_config, true);
#endif

    dma_channel_configure(dma_channel,
                          &dma_config,
                          &audio_pio->txf[sm],
                          NULL,
                          0,
                          false);

    irq_add_shared_handler(DMA_IRQ_0 + PICO_AUDIO_I2S_DMA_IRQ, audio_i2s_dma_irq_handler,
                           PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    dma_irqn_set_channel_enabled(PICO_AUDIO_I2S_DMA_IRQ, dma_channel, 1);
    return intended_audio_format;
}

static void update_pio_frequency(uint32_t sample_freq) {
    uint32_t system_clock_frequency = clock_get_hz(clk_sys);
    assert(system_clock_frequency < 0x40000000);
    // 32-bit I2S slots: 2 PIO cycles/bit × 32 bits/channel × 2 channels = 128 PIO cycles/frame
    // PIO clock = sample_freq × 128
    // Divider in 24.8 fixed point = sys_clock / (sample_freq × 128) × 256 = sys_clock × 2 / sample_freq
    uint32_t divider = system_clock_frequency * 2 / sample_freq;
    assert(divider < 0x1000000);
    pio_sm_set_clkdiv_int_frac(audio_pio, i2s_shared_state.pio_sm, divider >> 8u, divider & 0xffu);
    i2s_shared_state.freq = sample_freq;
}

static void stereo_s32_to_i2s_producer_give(audio_connection_t *connection, audio_buffer_t *buffer) {
    struct producer_pool_blocking_give_connection *pbc =
        (struct producer_pool_blocking_give_connection *)connection;

    uint32_t pos = 0;
    while (pos < buffer->sample_count) {
        if (!pbc->current_consumer_buffer) {
            pbc->current_consumer_buffer = get_free_audio_buffer(pbc->core.consumer_pool, true);
            pbc->current_consumer_buffer_pos = 0;
        }
        audio_buffer_t *con = pbc->current_consumer_buffer;
        uint32_t space = con->max_sample_count - pbc->current_consumer_buffer_pos;
        uint32_t avail = buffer->sample_count - pos;
        uint32_t n = (avail < space) ? avail : space;

        int32_t *src = (int32_t *)buffer->buffer->bytes + pos * 2;
        int32_t *dst = (int32_t *)con->buffer->bytes + pbc->current_consumer_buffer_pos * 2;

        // Left-justify 24-bit samples into 32-bit I2S slots
        for (uint32_t i = 0; i < n * 2; i++) {
            dst[i] = (int32_t)((uint32_t)src[i] << 8);
        }

        pos += n;
        pbc->current_consumer_buffer_pos += n;

        if (pbc->current_consumer_buffer_pos == con->max_sample_count) {
            con->sample_count = con->max_sample_count;
            queue_full_audio_buffer(pbc->core.consumer_pool, con);
            pbc->current_consumer_buffer = NULL;
        }
    }
    queue_free_audio_buffer(pbc->core.producer_pool, buffer);
}

static audio_buffer_t *wrap_consumer_take(audio_connection_t *connection, bool block) {
    if (connection->producer_pool->format->sample_freq != i2s_shared_state.freq) {
        update_pio_frequency(connection->producer_pool->format->sample_freq);
    }
    return consumer_pool_take_buffer_default(connection, block);
}

static struct producer_pool_blocking_give_connection i2s_pg_connection = {
    .core = {
        .consumer_pool_take = wrap_consumer_take,
        .consumer_pool_give = consumer_pool_give_buffer_default,
        .producer_pool_take = producer_pool_take_buffer_default,
        .producer_pool_give = stereo_s32_to_i2s_producer_give,
    }
};

static struct buffer_copying_on_consumer_take_connection i2s_ct_connection = {
    .core = {
        .consumer_pool_take = wrap_consumer_take,
        .consumer_pool_give = consumer_pool_give_buffer_default,
        .producer_pool_take = producer_pool_take_buffer_default,
        .producer_pool_give = producer_pool_give_buffer_default,
    }
};

bool audio_i2s_connect_extra(audio_buffer_pool_t *producer, bool buffer_on_give, uint buffer_count,
                             uint samples_per_buffer, audio_connection_t *connection) {
    assert(producer->format->format == AUDIO_BUFFER_FORMAT_PCM_S32);

    pio_i2s_consumer_format.format = AUDIO_BUFFER_FORMAT_PCM_S32;
    pio_i2s_consumer_format.sample_freq = producer->format->sample_freq;
    pio_i2s_consumer_format.channel_count = 2;
    // 32-bit stereo: 8 bytes per sample (2 × int32_t)
    pio_i2s_consumer_buffer_format.sample_stride = 8;

    audio_i2s_consumer = audio_new_consumer_pool(&pio_i2s_consumer_buffer_format, buffer_count, samples_per_buffer);

    update_pio_frequency(producer->format->sample_freq);

    __mem_fence_release();

    if (!connection) {
        connection = buffer_on_give ? &i2s_pg_connection.core : &i2s_ct_connection.core;
    }
    audio_complete_connection(connection, producer, audio_i2s_consumer);
    return true;
}

static inline void audio_start_dma_transfer() {
    assert(!i2s_shared_state.playing_buffer);
    audio_buffer_t *ab = take_audio_buffer(audio_i2s_consumer, false);

    i2s_shared_state.playing_buffer = ab;
    if (!ab) {
        ab = &silence_buffer;
    }
    assert(ab->sample_count);
    // 32-bit stereo: 2 DMA words (uint32_t) per sample
    uint32_t transfer_words = ab->sample_count * 2;
    i2s_shared_state.current_transfer_words = transfer_words;
    dma_channel_transfer_from_buffer_now(i2s_shared_state.dma_channel, ab->buffer->bytes, transfer_words);
}

void __isr __time_critical_func(audio_i2s_dma_irq_handler)() {
#if PICO_AUDIO_I2S_NOOP
    assert(false);
#else
    uint dma_channel = i2s_shared_state.dma_channel;
    if (dma_irqn_get_channel_status(PICO_AUDIO_I2S_DMA_IRQ, dma_channel)) {
        dma_irqn_acknowledge_channel(PICO_AUDIO_I2S_DMA_IRQ, dma_channel);
        i2s_shared_state.words_consumed += i2s_shared_state.current_transfer_words;
        if (i2s_shared_state.playing_buffer) {
            give_audio_buffer(audio_i2s_consumer, i2s_shared_state.playing_buffer);
            i2s_shared_state.playing_buffer = NULL;
        }
        audio_start_dma_transfer();
    }
#endif
}

static bool audio_enabled;

void audio_i2s_set_enabled(bool enabled) {
    if (enabled != audio_enabled) {
        irq_set_enabled(DMA_IRQ_0 + PICO_AUDIO_I2S_DMA_IRQ, enabled);
        if (enabled) {
            audio_start_dma_transfer();
        }
        pio_sm_set_enabled(audio_pio, i2s_shared_state.pio_sm, enabled);
        audio_enabled = enabled;
    }
}
