/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_AUDIO_I2S_H
#define _PICO_AUDIO_I2S_H

#include "pico/audio.h"

/** \file audio_i2s.h
 *  \defgroup pico_audio_i2s pico_audio_i2s
 *  I2S audio output using the PIO
 *
 * This library uses the \ref hardware_pio system to implement an I2S audio interface.
 * Supports 32-bit slots (24-bit audio data, left-justified).
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PICO_AUDIO_I2S_DMA_IRQ
#ifdef PICO_AUDIO_DMA_IRQ
#define PICO_AUDIO_I2S_DMA_IRQ PICO_AUDIO_DMA_IRQ
#else
#define PICO_AUDIO_I2S_DMA_IRQ 0
#endif
#endif

#ifndef PICO_AUDIO_I2S_PIO
#ifdef PICO_AUDIO_PIO
#define PICO_AUDIO_I2S_PIO PICO_AUDIO_PIO
#else
#define PICO_AUDIO_I2S_PIO 0
#endif
#endif

#ifndef PICO_AUDIO_I2S_BUFFER_SAMPLE_LENGTH
#ifdef PICO_AUDIO_BUFFER_SAMPLE_LENGTH
#define PICO_AUDIO_I2S_BUFFER_SAMPLE_LENGTH PICO_AUDIO_BUFFER_SAMPLE_LENGTH
#else
#define PICO_AUDIO_I2S_BUFFER_SAMPLE_LENGTH 192u
#endif
#endif

#ifndef PICO_AUDIO_I2S_SILENCE_BUFFER_SAMPLE_LENGTH
#define PICO_AUDIO_I2S_SILENCE_BUFFER_SAMPLE_LENGTH 192u
#endif

#ifndef PICO_AUDIO_I2S_NOOP
#ifdef PICO_AUDIO_NOOP
#define PICO_AUDIO_I2S_NOOP PICO_AUDIO_NOOP
#else
#define PICO_AUDIO_I2S_NOOP 0
#endif
#endif

#ifndef PICO_AUDIO_I2S_DATA_PIN
#define PICO_AUDIO_I2S_DATA_PIN 22
#endif

#ifndef PICO_AUDIO_I2S_CLOCK_PIN_BASE
#define PICO_AUDIO_I2S_CLOCK_PIN_BASE 26
#endif

typedef struct audio_i2s_config {
    uint8_t data_pin;
    uint8_t clock_pin_base;
    uint8_t dma_channel;
    uint8_t pio_sm;
} audio_i2s_config_t;

/** \brief Shared state for I2S output (exposed for USB feedback word tracking)
 */
typedef struct audio_i2s_state {
    audio_buffer_t *playing_buffer;
    uint32_t freq;
    uint8_t pio_sm;
    uint8_t dma_channel;
    volatile uint32_t words_consumed;
    uint32_t current_transfer_words;
} audio_i2s_state_t;

extern audio_i2s_state_t i2s_shared_state;

const audio_format_t *audio_i2s_setup(const audio_format_t *intended_audio_format,
                                      const audio_i2s_config_t *config);

bool audio_i2s_connect_extra(audio_buffer_pool_t *producer, bool buffer_on_give, uint buffer_count,
                             uint samples_per_buffer, audio_connection_t *connection);

void audio_i2s_set_enabled(bool enabled);

#ifdef __cplusplus
}
#endif

#endif //_AUDIO_I2S_H
