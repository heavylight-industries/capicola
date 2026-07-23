/**
 * @file hardware.h
 * @brief Panel constants for the Alchemy Lab module.
 *
 * NOTE (V2 / alchemy-sdk migration):
 *   Pin assignments, the WS2812 chain layout, the PCA9557 button expander,
 *   the MCP4728/STM-DAC CV-out matrix and CV calibration now live in the
 *   SDK board support package:
 *
 *       lib/alchemy-sdk/hardware/alchemy-lab/v2/include/alchemy/hw/alchemy_lab_v2_layout.h
 *
 *   The board class `alchemy::AlchemyLabV2` (see main.cpp) owns the pots,
 *   CV inputs, buttons (B1/B2 on-MCU, B3 via I²C expander) and the LED
 *   strip.  This header now only carries the plain panel/audio constants
 *   the UI + audio layers reference; it no longer hard-codes any GPIO.
 *
 *   The values below intentionally mirror the SDK's V2 layout constants so
 *   the existing control/animation code keeps compiling unchanged.
 */

#ifndef HARDWARE_H
#define HARDWARE_H

#include <cstdint>

/* ── Pots / CV / Buttons (counts only; pins come from the SDK BSP) ──────── */
constexpr uint8_t NUM_POTS     = 6;   /* == alchemy::kNumPots      */
constexpr uint8_t NUM_BUTTONS  = 3;   /* == alchemy::kNumButtons   */

constexpr uint8_t kNumCvInputs = 6;   /* == alchemy::kNumCvInputs  */
constexpr uint8_t kCvAdcOffset = NUM_POTS;

/* ── WS2812 LED panel geometry (matches the SDK V2 chain layout) ────────── */
constexpr uint8_t  LEDS_PER_RING    = 16;
constexpr uint8_t  NUM_LED_RINGS    = NUM_POTS;                          /* 6  */
constexpr uint8_t  LEDS_PER_BUTTON  = 2;                                 /* top+bottom */
constexpr uint16_t LED_RING_TOTAL   = LEDS_PER_RING * NUM_LED_RINGS;     /* 96 */
constexpr uint16_t LED_BUTTON_TOTAL = LEDS_PER_BUTTON * NUM_BUTTONS;     /* 6  */
constexpr uint16_t LED_TOTAL        = LED_RING_TOTAL + LED_BUTTON_TOTAL; /* 102 */

/* Hours per LED step around a ring (16 LEDs / 12 hours). */
constexpr float    LED_HOURS_PER_STEP = 12.0f / static_cast<float>(LEDS_PER_RING); /* 0.75 */

/* ── Audio ──────────────────────────────────────────────────────────────── */
/*
 * Block size is pinned to 64 (not the SDK's 24-sample default) so the audio
 * engine's one-block feedback buffers (AudioEngine::kMaxBlock) stay valid and
 * the DSP behaves byte-identically to the pre-V2 firmware.  main() passes this
 * to AlchemyLabV2::Init().
 */
constexpr uint32_t AUDIO_SAMPLE_RATE_HZ = 48000;
constexpr uint16_t AUDIO_BLOCK_SAMPLES  = 64;

#endif /* HARDWARE_H */
