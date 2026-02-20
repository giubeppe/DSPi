PROJECT(foxdac)

add_compile_options(-fcommon)
add_compile_options(-O2)

# Use -O3 for DSP-critical files
set_source_files_properties(
    dsp_pipeline.c usb_audio.c crossfeed.c loudness.c
    PROPERTIES COMPILE_FLAGS "-O3"
)

add_executable(foxdac
    config.h
    crossfeed.c
    crossfeed.h
    dcp_inline.h
    dsp_pipeline.c
    dsp_pipeline.h
    flash_storage.c
    flash_storage.h
    loudness.c
    loudness.h
    main.c
    pdm_generator.c
    pdm_generator.h
    usb_audio.c
    usb_audio.h
    usb_descriptors.c
    usb_descriptors.h
)

if (PICO_PLATFORM STREQUAL "rp2040")
    target_sources(foxdac PRIVATE dsp_process_rp2040.S)
endif()

pico_set_binary_type(foxdac copy_to_ram)

target_include_directories(foxdac PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/lufa
)

target_compile_definitions(foxdac PRIVATE
    AUDIO_FREQ_MAX=48000

    # S/PDIF output configuration (pins defined in config.h)
    PICO_AUDIO_SPDIF_PIO=0
    PICO_AUDIO_SPDIF_DMA_IRQ=1

    # I2S first output (PCM5102): PIO1 SM1, DMA4 (SPDIF uses 0-3)
    PICO_AUDIO_I2S_PIO=1
    PICO_AUDIO_I2S_DMA_IRQ=1
    PICO_AUDIO_I2S_DATA_PIN=22
    PICO_AUDIO_I2S_CLOCK_PIN_BASE=26

    PICO_DEFAULT_UART_TX_PIN=12

    # pico-extras usb_device configuration
    PICO_USBDEV_USE_ZERO_BASED_INTERFACES=1
    PICO_USBDEV_MAX_DESCRIPTOR_SIZE=256
    PICO_USBDEV_ISOCHRONOUS_BUFFER_STRIDE_TYPE=2
)

target_link_libraries(foxdac
    pico_stdlib
    pico_multicore
    pico_unique_id
    hardware_pwm
    hardware_flash
    hardware_adc
    pico_audio_spdif_multi
    pico_audio_i2s
    usb_device
)

pico_add_extra_outputs(foxdac)
