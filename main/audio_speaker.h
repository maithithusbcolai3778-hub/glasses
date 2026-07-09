/*
 * Audio speaker driver header for ESP32-P4-WIFI6-DEV-KIT-A
 *
 * Uses onboard ES8311 codec + NS4150B PA to drive the 8Ohm/2W speaker.
 * The driver is isolated from the camera/Wi-Fi code to avoid side effects.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the speaker subsystem (I2S, ES8311 codec, PA GPIO).
 *
 * This function reuses the existing I2C master bus used by the camera SCCB
 * if it already exists. If not, it creates one on the configured GPIOs.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t audio_speaker_init(void);

/**
 * @brief Play a short beep (1 kHz sine wave) to verify the speaker.
 *
 * The duration is controlled by CONFIG_AUDIO_SPEAKER_BEEP_DURATION_MS.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t audio_speaker_beep_test(void);

/**
 * @brief Play raw PCM data through the speaker.
 *
 * The data must be 16-bit mono PCM at CONFIG_AUDIO_SPEAKER_SAMPLE_RATE.
 *
 * @param pcm    Pointer to PCM buffer.
 * @param len    Length of the buffer in bytes.
 * @param timeout Maximum time to wait for the write to complete.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t audio_speaker_play(const uint8_t *pcm, size_t len, TickType_t timeout);

/**
 * @brief Set the speaker output volume.
 *
 * @param vol Volume level, 0 ~ 100.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t audio_speaker_set_volume(uint8_t vol);

/**
 * @brief Mute or unmute the speaker output.
 *
 * @param mute true to mute, false to unmute.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t audio_speaker_mute(bool mute);

/**
 * @brief Deinitialize the speaker subsystem.
 *
 * This releases I2S resources and closes the codec device. The I2C bus is
 * intentionally NOT deleted because it may be shared with the camera.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t audio_speaker_deinit(void);

#ifdef __cplusplus
}
#endif
