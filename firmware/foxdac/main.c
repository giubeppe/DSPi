/*
 * DSPi Main - USB Audio Device
 * USB Audio with DSP processing and S/PDIF output
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/unique_id.h"
#include "hardware/watchdog.h"
#include "hardware/vreg.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/timer.h"

// Local headers
#include "config.h"
#include "dsp_pipeline.h"
#include "flash_storage.h"
#include "pdm_generator.h"
#include "usb_audio.h"
#include "loudness.h"
#include "crossfeed.h"
#include "pico/audio_i2s.h"
#include "pico/audio_spdif.h"

// ----------------------------------------------------------------------------
// GLOBAL DEFINITIONS
// ----------------------------------------------------------------------------

// USB audio feedback (SOF-measured, 10.14 fixed-point)
volatile uint32_t feedback_10_14 = 0;
volatile uint32_t nominal_feedback_10_14 = 0;

// SOF handler state for feedback measurement
static volatile uint32_t feedback_reset_value = 0;

// USB SOF IRQ — provides async feedback to the host.
// Uses nominal (pre-computed) feedback from the sample rate, matching the
// original working implementation. The PIO clock is derived from the same
// PLL so the nominal value is accurate enough for USB rate adaptation.
void __not_in_flash_func(usb_sof_irq)(void) {
    if (feedback_reset_value) {
        feedback_10_14 = feedback_reset_value;
        feedback_reset_value = 0;
    } else if (feedback_10_14 == 0) {
        feedback_10_14 = nominal_feedback_10_14;
    }
}

volatile int overruns = 0;  // Legacy - kept for compatibility
volatile uint32_t pio_samples_dma = 0;

// Buffer monitoring counters
volatile uint32_t pdm_ring_overruns = 0;   // Core 0 couldn't push (ring full)
volatile uint32_t pdm_ring_underruns = 0;  // Core 1 needed sample but ring empty
volatile uint32_t pdm_dma_overruns = 0;    // Core 1 write caught up to DMA read
volatile uint32_t pdm_dma_underruns = 0;   // Core 1 write fell behind DMA read
volatile uint32_t spdif_overruns = 0;      // USB callback couldn't get buffer (pool full)
volatile uint32_t spdif_underruns = 0;     // USB packet gap > 2ms (consumer likely starved)
volatile uint32_t usb_audio_packets = 0;   // Debug: count of USB audio packets received
volatile uint32_t usb_audio_alt_set = 0;   // Debug: last alt setting selected
volatile uint32_t usb_audio_mounted = 0;   // Debug: audio mounted state
// Clock is fixed at 307.2MHz (no dynamic switching)

#include "pico/audio.h"
extern struct audio_format audio_format_48k;
extern MatrixMixer matrix_mixer;

static void perform_rate_change(uint32_t new_freq) {
    switch (new_freq) { case 44100: case 48000: case 96000: break; default: new_freq = 44100; }

    // Update the audio format so I2S and S/PDIF can update their PIO dividers
    audio_format_48k.sample_freq = new_freq;

    // 307.2MHz fixed clock — no switching needed.
    // 48kHz/96kHz get perfect integer PIO dividers;
    // 44.1kHz uses PIO fractional divider (acceptable jitter).

    // Reset sync
    extern volatile bool sync_started;
    extern volatile uint64_t total_samples_produced;
    sync_started = false;
    total_samples_produced = 0;

    // Pre-compute nominal feedback and reset SOF measurement
    nominal_feedback_10_14 = ((uint64_t)new_freq << 14) / 1000;
    feedback_10_14 = nominal_feedback_10_14;
    feedback_reset_value = nominal_feedback_10_14;

    dsp_recalculate_all_filters((float)new_freq);
    loudness_recompute_pending = true;
    crossfeed_update_pending = true;
    pdm_update_clock(new_freq);
}

void core0_init() {
    // LED setup
    gpio_init(25); gpio_set_dir(25, GPIO_OUT);

#if PICO_RP2350
    // RP2350: 307.2MHz — integer PIO dividers for 48kHz/96kHz
    vreg_set_voltage(VREG_VOLTAGE_1_10);
    busy_wait_ms(10);

    if (!set_sys_clock_hz(307200000, false)) {
        set_sys_clock_hz(150000000, false);
    }
#else
    // RP2040: 307.2MHz — VCO 1536MHz / 5 / 1
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    busy_wait_ms(10);
    set_sys_clock_pll(1536000000, 5, 1);
#endif

    // I2S MCLK output on a dedicated GPIO.
    // With clk_sys fixed at 307.2MHz, divide by 25 to get 12.288MHz (256×48kHz).
    // On 96kHz this is 128×fs, which most DACs accept as a valid MCLK ratio.
    clock_gpio_init_int_frac8(
        PICO_I2S_MCLK_PIN,
        CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS,
        25,
        0);

    gpio_init(23); gpio_set_dir(23, GPIO_OUT); gpio_put(23, 1);

    pico_get_unique_board_id_string(usb_descriptor_str_serial, 17);

    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    // [CRITICAL FIX]
    // Initialize USB/I2S/SPDIF *BEFORE* PDM.
    // I2S requires DMA Channel 0 (hardcoded in config).
    // If PDM inits first, it steals Ch 0 via dma_claim_unused_channel(), causing I2S to panic/crash.
    usb_sound_card_init();

    // Initialize nominal feedback for default sample rate
    nominal_feedback_10_14 = ((uint64_t)audio_state.freq << 14) / 1000;
    feedback_10_14 = nominal_feedback_10_14;

    // Try to load saved parameters from flash
    // If successful, this overwrites the defaults set by usb_sound_card_init()
    if (flash_load_params() == FLASH_OK) {
        uint32_t flags = save_and_disable_interrupts();
        dsp_recalculate_all_filters(48000.0f);
        dsp_update_delay_samples(48000.0f);
        restore_interrupts(flags);

        // Apply saved S/PDIF pin configuration (before Core 1 starts)
        // Index 0 in output_pins is I2S (fixed), SPDIF starts at index 1
        extern uint8_t output_pins[];
        extern audio_spdif_instance_t *spdif_instance_ptrs[];
        for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
            uint8_t pin_idx = i + 1;  // offset: output_pins[1] → spdif_instance_ptrs[0]
            if (output_pins[pin_idx] != spdif_instance_ptrs[i]->pin) {
                audio_spdif_set_enabled(spdif_instance_ptrs[i], false);
                audio_spdif_change_pin(spdif_instance_ptrs[i], output_pins[pin_idx]);
                audio_spdif_set_enabled(spdif_instance_ptrs[i], true);
            }
        }
    }

    // Initial loudness table computation (uses loaded or default params)
    loudness_recompute_table(loudness_ref_spl, loudness_intensity_pct, 48000.0f);
    if (loudness_enabled && loudness_active_table) {
        audio_set_volume(audio_state.volume);  // Re-select loudness coefficients
    }

#if ENABLE_SUB
    {
        extern uint8_t output_pins[];
        pdm_setup_hw(output_pins[NUM_PIN_OUTPUTS - 1]);
    }

    // Determine initial Core 1 mode from output enables (may have been loaded from flash)
    if (matrix_mixer.outputs[NUM_OUTPUT_CHANNELS - 1].enabled) {
        core1_mode = CORE1_MODE_PDM;
        pdm_set_enabled(true);
    } else {
        // Check if any outputs 2-7 are enabled for EQ worker
        bool any_eq_output = false;
        for (int i = CORE1_EQ_FIRST_OUTPUT; i <= CORE1_EQ_LAST_OUTPUT; i++) {
            if (matrix_mixer.outputs[i].enabled) { any_eq_output = true; break; }
        }
        core1_mode = any_eq_output ? CORE1_MODE_EQ_WORKER : CORE1_MODE_IDLE;
        pdm_set_enabled(false);
    }

    multicore_launch_core1(pdm_core1_entry);
#endif
}

int main(void) {
    // Initial LED on to show we're alive
    gpio_init(25); gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 1);

#if !PICO_RP2350
    // Temporary safe clock before core0_init sets 307.2MHz
    set_sys_clock_pll(1536000000, 6, 2);
#endif

    core0_init();

    // Enable watchdog
    watchdog_enable(8000, 1);

    while (1) {
        // Update watchdog
        watchdog_update();

        // Handle EQ parameter updates from USB
        if (eq_update_pending) {
            EqParamPacket p = pending_packet;
            eq_update_pending = false;
            filter_recipes[p.channel][p.band] = p;

            // If updating a Core 1 EQ channel, wait for Core 1 to finish
            // current work before modifying coefficients
            bool is_core1_channel = (p.channel >= (CH_OUT_1 + CORE1_EQ_FIRST_OUTPUT) &&
                                     p.channel <= (CH_OUT_1 + CORE1_EQ_LAST_OUTPUT));
            if (is_core1_channel && core1_mode == CORE1_MODE_EQ_WORKER) {
                // Spin-wait until Core 1 is idle (work_done or no work dispatched)
                while (core1_eq_work.work_ready && !core1_eq_work.work_done) {
                    tight_loop_contents();
                }
                __dmb();
            }

            uint32_t flags = save_and_disable_interrupts();
            dsp_compute_coefficients(&p, &filters[p.channel][p.band], (float)audio_state.freq);

            // Recalculate channel bypass flag
            bool all_bypassed = true;
            for (int b = 0; b < channel_band_counts[p.channel]; b++) {
                if (!filters[p.channel][b].bypass) {
                    all_bypassed = false;
                    break;
                }
            }
            channel_bypassed[p.channel] = all_bypassed;

            restore_interrupts(flags);
        }

        // Handle sample rate changes
        if (rate_change_pending) {
            uint32_t r = pending_rate;
            rate_change_pending = false;
            perform_rate_change(r);
        }

        // Handle loudness table recomputation
        if (loudness_recompute_pending) {
            loudness_recompute_pending = false;
            loudness_recompute_table(loudness_ref_spl, loudness_intensity_pct, (float)audio_state.freq);
            // Update coefficient pointer for current volume
            if (loudness_enabled && loudness_active_table) {
                audio_set_volume(audio_state.volume);
            }
        }

        // Handle crossfeed coefficient updates
        if (crossfeed_update_pending) {
            crossfeed_update_pending = false;
            crossfeed_compute_coefficients(&crossfeed_state, (const CrossfeedConfig *)&crossfeed_config, (float)audio_state.freq);
            // Update bypass flag atomically
            crossfeed_bypassed = !crossfeed_config.enabled;
        }

        // LED heartbeat - toggle every ~1000 iterations
        static uint32_t loop_counter = 0;
        if (++loop_counter >= 1000) {
            loop_counter = 0;
            gpio_xor_mask(1u << 25);
        }
    }
}
