/*
 * SD card implementation for ESP32-P4-WIFI6-DEV-KIT onboard TF card slot.
 */

#include "sd_card.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_default_configs.h"

/* ESP32-P4 SDIO Slot 0 IOs are powered by an on-chip LDO (default channel 4).
 * This must be enabled for reliable high-speed SD card writes. */
#if SOC_SDMMC_IO_POWER_EXTERNAL
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#endif

#if !CONFIG_SD_CARD_ENABLE
/* Stubs used when SD card support is disabled in menuconfig. */

esp_err_t sd_card_init(void)
{
    return ESP_OK;
}

esp_err_t sd_card_start_monitor_task(void)
{
    return ESP_OK;
}

bool sd_card_is_mounted(void)
{
    return false;
}

esp_err_t sd_card_get_free_space(uint64_t *out_free_bytes,
                                 uint64_t *out_total_bytes)
{
    return ESP_ERR_INVALID_STATE;
}

esp_err_t sd_card_save_photo(const uint8_t *jpeg_data,
                             size_t jpeg_len,
                             char *out_path,
                             size_t path_len)
{
    return ESP_ERR_INVALID_STATE;
}

const char *sd_card_get_photo_dir(void)
{
    return NULL;
}

esp_err_t sd_card_deinit(void)
{
    return ESP_OK;
}

esp_err_t sd_card_save_photo_to_path(const uint8_t *jpeg_data,
                                     size_t jpeg_len,
                                     const char *path)
{
    return ESP_ERR_INVALID_STATE;
}

void sd_card_generate_photo_path(char *out_path, size_t path_len)
{
    if (out_path && path_len > 0) {
        out_path[0] = '\0';
    }
}

#else /* CONFIG_SD_CARD_ENABLE */

/* ESP-Hosted (on-board ESP32-C6 Wi-Fi) already manages the shared SDMMC host
 * for SDIO Slot 1. When SDIO is used, prevent the SD card mount helper from
 * re-initialising or de-initialising that shared host. */
#if CONFIG_ESP_HOSTED_SDIO_HOST_INTERFACE
static esp_err_t sd_card_host_init_dummy(void)
{
    return ESP_OK;
}

static esp_err_t sd_card_host_deinit_dummy(void)
{
    return ESP_OK;
}
#endif

#define TAG "sd_card"

#define SD_MOUNT_POINT      CONFIG_SD_CARD_MOUNT_POINT
#define SD_PHOTO_DIR        CONFIG_SD_CARD_PHOTO_DIR
#define SD_MAX_FILES        5

static sdmmc_card_t *s_card = NULL;
static bool s_mounted = false;
static uint32_t s_photo_counter = 0;

#if SOC_SDMMC_IO_POWER_EXTERNAL
static sd_pwr_ctrl_handle_t s_pwr_ctrl_handle = NULL;
#endif

static const char *s_photo_dir = SD_MOUNT_POINT "/" SD_PHOTO_DIR;

const char *sd_card_get_photo_dir(void)
{
    return s_photo_dir;
}

bool sd_card_is_mounted(void)
{
    return s_mounted;
}

esp_err_t sd_card_get_free_space(uint64_t *out_free_bytes,
                                 uint64_t *out_total_bytes)
{
    if (!s_mounted) {
        return ESP_ERR_INVALID_STATE;
    }

    FATFS *fs = NULL;
    DWORD free_clusters = 0;
    FRESULT res = f_getfree(SD_MOUNT_POINT, &free_clusters, &fs);
    if (res != FR_OK) {
        ESP_LOGE(TAG, "f_getfree failed: %d", (int)res);
        return ESP_FAIL;
    }

    if (fs) {
        uint64_t total_sectors = (uint64_t)(fs->n_fatent - 2) * fs->csize;
        uint64_t free_sectors = (uint64_t)free_clusters * fs->csize;
        uint64_t sector_size = (uint64_t)fs->ssize;

        if (out_total_bytes) {
            *out_total_bytes = total_sectors * sector_size;
        }
        if (out_free_bytes) {
            *out_free_bytes = free_sectors * sector_size;
        }
    }

    return ESP_OK;
}

esp_err_t sd_card_init(void)
{
    if (s_mounted) {
        return ESP_OK;
    }

    if (!CONFIG_SD_CARD_ENABLE) {
        ESP_LOGW(TAG, "SD card disabled in menuconfig");
        return ESP_OK;
    }

    esp_err_t ret;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = SD_MAX_FILES,
        .allocation_unit_size = 16 * 1024,
        .use_one_fat = false,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;
#if CONFIG_ESP_HOSTED_SDIO_HOST_INTERFACE
    /* The shared SDMMC host is owned by ESP-Hosted for the SDIO link to the
     * ESP32-C6. Do not let the FAT mount helper init/deinit it. */
    host.init = sd_card_host_init_dummy;
    host.deinit = sd_card_host_deinit_dummy;
#endif
    host.max_freq_khz = CONFIG_SD_CARD_FREQ_KHZ;

#if SOC_SDMMC_IO_POWER_EXTERNAL
    /* Enable the on-chip LDO that powers SDIO Slot 0 IOs.
     * Without this, high-speed writes can timeout with 0x107. */
    if (s_pwr_ctrl_handle == NULL) {
        sd_pwr_ctrl_ldo_config_t ldo_config = {
            .ldo_chan_id = 4,
        };
        ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &s_pwr_ctrl_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SDIO LDO init failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    host.pwr_ctrl_handle = s_pwr_ctrl_handle;
#endif

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.clk = CONFIG_SD_CARD_GPIO_CLK;
    slot_config.cmd = CONFIG_SD_CARD_GPIO_CMD;
    slot_config.d0  = CONFIG_SD_CARD_GPIO_D0;
    slot_config.d1  = CONFIG_SD_CARD_GPIO_D1;
    slot_config.d2  = CONFIG_SD_CARD_GPIO_D2;
    slot_config.d3  = CONFIG_SD_CARD_GPIO_D3;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    ESP_LOGI(TAG, "mounting SD card...");
    ESP_LOGI(TAG, "clk=%d cmd=%d d0=%d d1=%d d2=%d d3=%d freq=%dkHz",
             slot_config.clk, slot_config.cmd, slot_config.d0,
             slot_config.d1, slot_config.d2, slot_config.d3,
             host.max_freq_khz);

    ret = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host,
                                  &slot_config, &mount_config,
                                  &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD card mount failed: %s", esp_err_to_name(ret));
        s_card = NULL;
        s_mounted = false;
        return ret;
    }

    s_mounted = true;
    sdmmc_card_print_info(stdout, s_card);

    /* Create photo directory if it does not exist. */
    struct stat st;
    if (stat(s_photo_dir, &st) != 0) {
        ESP_LOGI(TAG, "creating photo dir: %s", s_photo_dir);
        if (mkdir(s_photo_dir, 0755) != 0) {
            ESP_LOGE(TAG, "failed to create photo dir");
            /* Continue anyway; save will fail later if dir missing. */
        }
    }

    uint64_t free_bytes = 0;
    uint64_t total_bytes = 0;
    if (sd_card_get_free_space(&free_bytes, &total_bytes) == ESP_OK) {
        ESP_LOGI(TAG, "SD free space: %.2f GB / %.2f GB",
                 (double)free_bytes / (1024.0 * 1024.0 * 1024.0),
                 (double)total_bytes / (1024.0 * 1024.0 * 1024.0));
    }

    ESP_LOGI(TAG, "SD card init success");
    return ESP_OK;
}

static void sd_card_monitor_task(void *arg)
{
    (void)arg;
    /* Only retry a limited number of times at boot. Continuous polling of a
     * missing SD card uses sdmmc_card_init on slot 0, which can interfere with
     * ESP-Hosted SDIO on slot 1. */
    int retries_left = 12;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (s_mounted) {
            continue;
        }
        if (!CONFIG_SD_CARD_ENABLE) {
            continue;
        }
        if (retries_left <= 0) {
            ESP_LOGI(TAG, "SD card monitor: giving up after boot retries");
            break;
        }
        retries_left--;
        ESP_LOGI(TAG, "SD card monitor: retrying mount...");
        if (sd_card_init() == ESP_OK) {
            ESP_LOGI(TAG, "SD card monitor: mount succeeded");
        }
    }
    vTaskDelete(NULL);
}

esp_err_t sd_card_start_monitor_task(void)
{
    if (!CONFIG_SD_CARD_ENABLE) {
        return ESP_OK;
    }
    BaseType_t ret = xTaskCreate(sd_card_monitor_task,
                                 "sd_card_monitor",
                                 4096,
                                 NULL,
                                 3,
                                 NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "failed to create SD card monitor task");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void sd_card_build_photo_path(char *path, size_t path_len)
{
    /* Try RTC time first, fallback to monotonic counter. */
    time_t now = 0;
    struct tm timeinfo = {0};
    time(&now);
    localtime_r(&now, &timeinfo);

    if (timeinfo.tm_year > (2020 - 1900)) {
        snprintf(path, path_len,
                 "%s/IMG_%04d%02d%02d_%02d%02d%02d.jpg",
                 s_photo_dir,
                 timeinfo.tm_year + 1900,
                 timeinfo.tm_mon + 1,
                 timeinfo.tm_mday,
                 timeinfo.tm_hour,
                 timeinfo.tm_min,
                 timeinfo.tm_sec);
    } else {
        s_photo_counter++;
        snprintf(path, path_len, "%s/IMG_%05lu.jpg", s_photo_dir,
                 (unsigned long)s_photo_counter);
    }
}

void sd_card_generate_photo_path(char *out_path, size_t path_len)
{
    if (!out_path || path_len == 0) {
        return;
    }
    sd_card_build_photo_path(out_path, path_len);
}

esp_err_t sd_card_save_photo_to_path(const uint8_t *jpeg_data,
                                     size_t jpeg_len,
                                     const char *path)
{
    ESP_RETURN_ON_FALSE(jpeg_data != NULL, ESP_ERR_INVALID_ARG, TAG, "jpeg_data is null");
    ESP_RETURN_ON_FALSE(jpeg_len > 0, ESP_ERR_INVALID_ARG, TAG, "jpeg_len is 0");
    ESP_RETURN_ON_FALSE(path != NULL, ESP_ERR_INVALID_ARG, TAG, "path is null");
    ESP_RETURN_ON_FALSE(s_mounted, ESP_ERR_INVALID_STATE, TAG, "SD card not mounted");

    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "failed to open file: %s", path);
        return ESP_FAIL;
    }

    size_t written = fwrite(jpeg_data, 1, jpeg_len, f);
    fclose(f);

    if (written != jpeg_len) {
        ESP_LOGE(TAG, "failed to write complete file: %zu/%zu", written, jpeg_len);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "photo saved: %s (%zu bytes)", path, jpeg_len);
    return ESP_OK;
}

esp_err_t sd_card_save_photo(const uint8_t *jpeg_data,
                             size_t jpeg_len,
                             char *out_path,
                             size_t path_len)
{
    ESP_RETURN_ON_FALSE(jpeg_data != NULL, ESP_ERR_INVALID_ARG, TAG, "jpeg_data is null");
    ESP_RETURN_ON_FALSE(jpeg_len > 0, ESP_ERR_INVALID_ARG, TAG, "jpeg_len is 0");
    ESP_RETURN_ON_FALSE(s_mounted, ESP_ERR_INVALID_STATE, TAG, "SD card not mounted");

    char path[128];
    sd_card_build_photo_path(path, sizeof(path));

    esp_err_t ret = sd_card_save_photo_to_path(jpeg_data, jpeg_len, path);
    if (ret == ESP_OK && out_path && path_len > 0) {
        strncpy(out_path, path, path_len - 1);
        out_path[path_len - 1] = '\0';
    }
    return ret;
}

esp_err_t sd_card_deinit(void)
{
    if (!s_mounted) {
        return ESP_OK;
    }

    esp_err_t ret = esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
    s_card = NULL;
    s_mounted = false;

#if SOC_SDMMC_IO_POWER_EXTERNAL
    if (s_pwr_ctrl_handle != NULL) {
        sd_pwr_ctrl_del_on_chip_ldo(s_pwr_ctrl_handle);
        s_pwr_ctrl_handle = NULL;
    }
#endif

    return ret;
}

#endif /* CONFIG_SD_CARD_ENABLE */
