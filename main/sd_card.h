/*
 * SD card support for ESP32-P4-WIFI6-DEV-KIT onboard TF card slot.
 *
 * Uses SDMMC 4-bit mode with the Waveshare-recommended pin mapping:
 *   CLK = GPIO43, CMD = GPIO44,
 *   D0 = GPIO39, D1 = GPIO40, D2 = GPIO41, D3 = GPIO42
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize and mount the onboard SD card.
 *
 * Creates the photo directory if it does not exist.
 *
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t sd_card_init(void);

/**
 * @brief Start a background task that retries SD card mount every 5 s.
 *
 * This allows the card to be detected after hot-plug.
 */
esp_err_t sd_card_start_monitor_task(void);

/**
 * @brief Return whether the SD card was successfully mounted.
 */
bool sd_card_is_mounted(void);

/**
 * @brief Get free space on the SD card in bytes.
 *
 * @param out_free_bytes  output, may be NULL.
 * @param out_total_bytes output, may be NULL.
 * @return ESP_OK on success.
 */
esp_err_t sd_card_get_free_space(uint64_t *out_free_bytes,
                                 uint64_t *out_total_bytes);

/**
 * @brief Save a JPEG buffer to the SD card photo directory.
 *
 * The filename is generated from the current RTC time if available,
 * otherwise a monotonic counter is used.
 *
 * @param jpeg_data  JPEG data pointer.
 * @param jpeg_len   JPEG data length.
 * @param out_path   Buffer to receive the full saved path. May be NULL.
 * @param path_len   Size of out_path buffer.
 * @return ESP_OK on success.
 */
esp_err_t sd_card_save_photo(const uint8_t *jpeg_data,
                             size_t jpeg_len,
                             char *out_path,
                             size_t path_len);

/**
 * @brief Save a JPEG buffer to a specific file path on the SD card.
 */
esp_err_t sd_card_save_photo_to_path(const uint8_t *jpeg_data,
                                     size_t jpeg_len,
                                     const char *path);

/**
 * @brief Generate the next photo filename (without writing).
 */
void sd_card_generate_photo_path(char *out_path, size_t path_len);

/**
 * @brief Return the configured photo directory path.
 */
const char *sd_card_get_photo_dir(void);

/**
 * @brief Unmount the SD card and release resources.
 */
esp_err_t sd_card_deinit(void);

#ifdef __cplusplus
}
#endif
