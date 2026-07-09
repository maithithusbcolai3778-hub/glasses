/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include "esp_http_server.h"
#include "mdns.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"
#ifndef CONFIG_CAMERA_OV5647_CSI_LINESYNC_ENABLE
#define CONFIG_CAMERA_OV5647_CSI_LINESYNC_ENABLE 0
#endif

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/jpeg_encode.h"
#include "driver/jpeg_types.h"
#include "audio_speaker.h"
#include <math.h>
#include "imu_atk_ms601m.h"
#include "sd_card.h"
#include "imu_html.h"


#include "esp_cam_sensor_types.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
#include "esp_video_ioctl.h"
#include "linux/videodev2.h"
#include "linux/v4l2-common.h"



/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

#define IMU_LOG_INTERVAL_MS 1000

static void imu_log_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(IMU_LOG_INTERVAL_MS));

        imu_attitude_t att = {0};
        imu_stats_t stats = {0};
        if (imu_atk_ms601m_get_attitude(&att) == ESP_OK &&
            imu_atk_ms601m_get_stats(&stats) == ESP_OK) {
            ESP_LOGI(TAG, "IMU roll=%.2f pitch=%.2f yaw=%.2f | frames=%u bytes=%u err=%u",
                     att.roll, att.pitch, att.yaw,
                     (unsigned)stats.frames_parsed,
                     (unsigned)stats.bytes_received,
                     (unsigned)stats.checksum_errors);
        }
    }
}


/* IMU turn detection: uses gyroscope yaw rate and accumulated yaw change. */
#define TURN_SAMPLE_PERIOD_MS       50
#define TURN_GYRO_THRESHOLD_DPS     15.0f
#define TURN_START_COUNT            3
#define TURN_END_COUNT              5
#define TURN_MIN_ANGLE_DEG          25.0f

typedef enum {
    TURN_DIR_NONE = 0,
    TURN_DIR_LEFT,
    TURN_DIR_RIGHT
} turn_dir_t;

static SemaphoreHandle_t s_turn_mutex = NULL;
static volatile bool s_turn_in_progress = false;
static volatile turn_dir_t s_turn_dir = TURN_DIR_NONE;
static volatile float s_turn_progress_deg = 0.0f;
static volatile float s_last_completed_turn_deg = 0.0f;
static volatile uint32_t s_turn_completed_count = 0;
static char s_turn_state_str[32] = "idle";

static float normalize_yaw_delta(float delta)
{
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    return delta;
}

static const char *turn_dir_name(turn_dir_t dir)
{
    switch (dir) {
        case TURN_DIR_LEFT:  return "left";
        case TURN_DIR_RIGHT: return "right";
        default:             return "none";
    }
}

static void imu_turn_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "imu_turn_task started");

    float last_yaw = 0.0f;
    bool have_last_yaw = false;
    int over_threshold_count = 0;
    int below_threshold_count = 0;
    bool pending_turn = false;
    float start_yaw = 0.0f;
    turn_dir_t pending_dir = TURN_DIR_NONE;
    float accumulated_deg = 0.0f;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(TURN_SAMPLE_PERIOD_MS));

        imu_attitude_t att = {0};
        imu_motion_t motion = {0};
        if (imu_atk_ms601m_get_attitude(&att) != ESP_OK ||
            imu_atk_ms601m_get_motion(&motion) != ESP_OK) {
            continue;
        }

        float gyro_z = motion.gyro_dps[2];
        float yaw = att.yaw;

        if (!have_last_yaw) {
            last_yaw = yaw;
            have_last_yaw = true;
            continue;
        }

        float yaw_delta = normalize_yaw_delta(yaw - last_yaw);
        last_yaw = yaw;

        if (fabsf(gyro_z) > TURN_GYRO_THRESHOLD_DPS) {
            over_threshold_count++;
            below_threshold_count = 0;

            if (!pending_turn && over_threshold_count >= TURN_START_COUNT) {
                pending_turn = true;
                pending_dir = (gyro_z > 0.0f) ? TURN_DIR_LEFT : TURN_DIR_RIGHT;
                start_yaw = yaw;
                accumulated_deg = 0.0f;
            }

            if (pending_turn) {
                accumulated_deg += yaw_delta * ((pending_dir == TURN_DIR_LEFT) ? 1.0f : -1.0f);
                if (accumulated_deg < 0.0f) accumulated_deg = 0.0f;
            }
        } else {
            below_threshold_count++;
            if (below_threshold_count >= TURN_END_COUNT) {
                over_threshold_count = 0;
            }

            if (pending_turn && below_threshold_count >= TURN_END_COUNT) {
                float total = fabsf(normalize_yaw_delta(yaw - start_yaw));

                if (xSemaphoreTake(s_turn_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    s_turn_in_progress = false;
                    s_turn_progress_deg = 0.0f;

                    if (total >= TURN_MIN_ANGLE_DEG) {
                        s_turn_dir = pending_dir;
                        s_last_completed_turn_deg = total;
                        s_turn_completed_count++;
                        snprintf(s_turn_state_str, sizeof(s_turn_state_str),
                                 "%s %.0f deg", turn_dir_name(pending_dir), total);
                        ESP_LOGI(TAG, "turn completed: %s %.1f deg (accumulated %.1f)",
                                 turn_dir_name(pending_dir), total, accumulated_deg);
                    } else {
                        snprintf(s_turn_state_str, sizeof(s_turn_state_str), "idle");
                    }
                    xSemaphoreGive(s_turn_mutex);
                }

                pending_turn = false;
                pending_dir = TURN_DIR_NONE;
                accumulated_deg = 0.0f;
            }
        }

        if (pending_turn) {
            if (xSemaphoreTake(s_turn_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                s_turn_in_progress = true;
                s_turn_dir = pending_dir;
                s_turn_progress_deg = accumulated_deg;
                snprintf(s_turn_state_str, sizeof(s_turn_state_str),
                         "turning %s", turn_dir_name(pending_dir));
                xSemaphoreGive(s_turn_mutex);
            }
        }
    }
}


#define CAMERA_BUF_COUNT     4
#define CAMERA_EMPTY_POLL_LIMIT 100


static SemaphoreHandle_t s_frame_mutex = NULL;
static uint8_t *s_latest_frame = NULL;
static size_t s_latest_frame_len = 0;
static uint32_t s_latest_frame_width = CONFIG_CAM_H_RES;
static uint32_t s_latest_frame_height = CONFIG_CAM_V_RES;
static uint32_t s_latest_frame_pixfmt = V4L2_PIX_FMT_RGB565;
static uint32_t s_latest_frame_bytesperline = 0;
static uint32_t s_latest_frame_id = 0;
static size_t s_latest_frame_capacity = 0;

#define JPEG_PREVIEW_QUALITY          30
#define JPEG_PREVIEW_PERIOD_MS        66   
#define MJPEG_BOUNDARY                "frame"

static SemaphoreHandle_t s_jpeg_mutex = NULL;
static uint8_t *s_latest_jpeg = NULL;
static size_t s_latest_jpeg_len = 0;
static size_t s_latest_jpeg_capacity = 0;
static uint32_t s_latest_jpeg_width = 0;
static uint32_t s_latest_jpeg_height = 0;
static uint32_t s_latest_jpeg_id = 0;
static uint32_t s_latest_jpeg_source_frame_id = 0;
static uint32_t s_jpeg_encode_error_count = 0;
static uint32_t s_jpeg_encode_ok_count = 0;
static uint32_t s_jpeg_last_size = 0;
static uint32_t s_jpeg_last_ms = 0;
static char s_jpeg_state[64] = "not started";
static volatile uint32_t s_camera_timeout_count = 0;
static volatile uint32_t s_camera_dqbuf_error_count = 0;
static char s_camera_device_name[32] = "none";
static char s_camera_state[64] = "not started";
static int s_next_camera_device = 0;
static bool s_camera_force_no_linesync = false;

typedef struct {
    void *start;
    size_t length;
} camera_buffer_t;

static int s_retry_num = 0;
static char s_board_ip[16] = "0.0.0.0";
static SemaphoreHandle_t s_speaker_mutex = NULL;
static volatile TickType_t s_last_play_ticks = 0;
static esp_netif_t *s_sta_netif = NULL;

/* Audio playback queue: HTTP handler posts PCM here, a dedicated task plays it.
 * This prevents long-running audio_speaker_play from blocking the HTTP server task. */
#define AUDIO_QUEUE_LEN 8
typedef struct {
    uint8_t *pcm;
    size_t len;
} audio_queue_item_t;
static QueueHandle_t s_audio_queue = NULL;

/* HTTP watchdog: reboot if the server has stopped serving requests. */
#define HTTP_WATCHDOG_TIMEOUT_MS 120000
static volatile TickType_t s_last_http_request_ticks = 0;
static volatile uint32_t s_http_request_count = 0;
#define HTTP_REQ_ENTER() do { \
    s_last_http_request_ticks = xTaskGetTickCount(); \
    s_http_request_count++; \
} while (0)

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        snprintf(s_board_ip, sizeof(s_board_ip), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Camera web URL: http://%s/", s_board_ip);
        ESP_LOGI(TAG, "MJPEG stream URL: http://%s/stream", s_board_ip);
        ESP_LOGI(TAG, "Status URL: http://%s/status", s_board_ip);
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s", EXAMPLE_ESP_WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", EXAMPLE_ESP_WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

static void i2c_scan_camera_bus(gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    ESP_LOGI(TAG, "Scanning I2C bus: SDA=%d SCL=%d", sda_gpio, scl_gpio);

    i2c_master_bus_handle_t bus = NULL;

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = CONFIG_OV5647_SCCB_I2C_PORT,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(ret));
        return;
    }

    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        ret = i2c_master_probe(bus, addr, 50);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found at 0x%02x", addr);
            found++;
        }
    }

    if (found == 0) {
        ESP_LOGW(TAG, "No I2C device found on SDA=%d SCL=%d", sda_gpio, scl_gpio);
    }

    i2c_del_master_bus(bus);
}

static esp_err_t camera_read_reg16(i2c_master_dev_handle_t dev, uint16_t reg, uint8_t *value)
{
    uint8_t reg_buf[2] = {
        (uint8_t)(reg >> 8),
        (uint8_t)(reg & 0xff),
    };

    return i2c_master_transmit_receive(dev, reg_buf, sizeof(reg_buf), value, 1, 100);
}

static esp_err_t camera_write_reg16(i2c_master_dev_handle_t dev, uint16_t reg, uint8_t value)
{
    uint8_t buf[3] = {
        (uint8_t)(reg >> 8),
        (uint8_t)(reg & 0xff),
        value,
    };

    return i2c_master_transmit(dev, buf, sizeof(buf), 100);
}

static esp_err_t ov5647_with_i2c_device(esp_err_t (*fn)(i2c_master_dev_handle_t dev, void *ctx), void *ctx)
{
    i2c_master_bus_handle_t bus = NULL;
    i2c_master_dev_handle_t dev = NULL;
    bool created_bus = false;

    esp_err_t ret = i2c_master_get_bus_handle(CONFIG_OV5647_SCCB_I2C_PORT, &bus);
    if (ret != ESP_OK) {
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port = CONFIG_OV5647_SCCB_I2C_PORT,
            .sda_io_num = CONFIG_OV5647_SCCB_SDA_GPIO,
            .scl_io_num = CONFIG_OV5647_SCCB_SCL_GPIO,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };

        ret = i2c_new_master_bus(&bus_cfg, &bus);
        if (ret != ESP_OK) {
            return ret;
        }
        created_bus = true;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x36,
        .scl_speed_hz = CONFIG_OV5647_SCCB_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    if (ret == ESP_OK) {
        ret = fn(dev, ctx);
        i2c_master_bus_rm_device(dev);
    }

    if (created_bus) {
        i2c_del_master_bus(bus);
    }

    return ret;
}

static esp_err_t ov5647_force_mipi_ctrl_cb(i2c_master_dev_handle_t dev, void *ctx)
{
    (void)ctx;
    return camera_write_reg16(dev, 0x4800, 0x00);
}

static void ov5647_force_mipi_ctrl_no_linesync(void)
{
    esp_err_t ret = ov5647_with_i2c_device(ov5647_force_mipi_ctrl_cb, NULL);
    ESP_LOGI(TAG, "OV5647 force MIPI ctrl 0x4800=0x00 ret=%s", esp_err_to_name(ret));
}

static void ov5647_log_stream_regs(const char *stage)
{
    i2c_master_bus_handle_t bus = NULL;
    i2c_master_dev_handle_t dev = NULL;
    bool created_bus = false;

    esp_err_t ret = i2c_master_get_bus_handle(CONFIG_OV5647_SCCB_I2C_PORT, &bus);
    if (ret != ESP_OK) {
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port = CONFIG_OV5647_SCCB_I2C_PORT,
            .sda_io_num = CONFIG_OV5647_SCCB_SDA_GPIO,
            .scl_io_num = CONFIG_OV5647_SCCB_SCL_GPIO,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };

        ret = i2c_new_master_bus(&bus_cfg, &bus);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "OV5647 regs %s: get/create i2c bus failed: %s", stage, esp_err_to_name(ret));
            return;
        }
        created_bus = true;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x36,
        .scl_speed_hz = CONFIG_OV5647_SCCB_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "OV5647 regs %s: add device failed: %s", stage, esp_err_to_name(ret));
        if (created_bus) {
            i2c_del_master_bus(bus);
        }
        return;
    }

    uint8_t mode = 0;
    uint8_t mipi = 0;
    uint8_t lane = 0;
    esp_err_t ret_mode = camera_read_reg16(dev, 0x0100, &mode);
    esp_err_t ret_mipi = camera_read_reg16(dev, 0x4800, &mipi);
    esp_err_t ret_lane = camera_read_reg16(dev, 0x3018, &lane);

    ESP_LOGI(TAG, "OV5647 regs %s: 0100=%s/0x%02x 4800=%s/0x%02x 3018=%s/0x%02x",
             stage,
             esp_err_to_name(ret_mode), mode,
             esp_err_to_name(ret_mipi), mipi,
             esp_err_to_name(ret_lane), lane);

    i2c_master_bus_rm_device(dev);
    if (created_bus) {
        i2c_del_master_bus(bus);
    }
}

static void camera_probe_sensor_id(gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    ESP_LOGI(TAG, "Probing camera sensor ID on SDA=%d SCL=%d", sda_gpio, scl_gpio);

    i2c_master_bus_handle_t bus = NULL;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = CONFIG_OV5647_SCCB_I2C_PORT,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "sensor id probe: i2c_new_master_bus failed: %s", esp_err_to_name(ret));
        return;
    }

    const struct {
        const char *name;
        uint8_t addr;
        uint16_t id_high_reg;
        uint16_t id_low_reg;
        uint16_t expected_id;
    } probes[] = {
        {"OV5647", 0x36, 0x300a, 0x300b, 0x5647},
        {"IMX219", 0x10, 0x0000, 0x0001, 0x0219},
    };

    for (int i = 0; i < (int)(sizeof(probes) / sizeof(probes[0])); i++) {
        i2c_master_dev_handle_t dev = NULL;
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = probes[i].addr,
            .scl_speed_hz = CONFIG_OV5647_SCCB_FREQ_HZ,
        };

        ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "sensor id probe: add %s addr 0x%02x failed: %s",
                     probes[i].name, probes[i].addr, esp_err_to_name(ret));
            continue;
        }

        uint8_t high = 0;
        uint8_t low = 0;
        esp_err_t ret_high = camera_read_reg16(dev, probes[i].id_high_reg, &high);
        esp_err_t ret_low = camera_read_reg16(dev, probes[i].id_low_reg, &low);
        if (ret_high == ESP_OK && ret_low == ESP_OK) {
            uint16_t id = ((uint16_t)high << 8) | low;
            ESP_LOGI(TAG, "sensor id probe: %s addr=0x%02x id=0x%04x expected=0x%04x %s",
                     probes[i].name, probes[i].addr, id, probes[i].expected_id,
                     id == probes[i].expected_id ? "MATCH" : "MISMATCH");
        } else {
            ESP_LOGW(TAG, "sensor id probe: %s addr=0x%02x read failed high=%s low=%s",
                     probes[i].name, probes[i].addr,
                     esp_err_to_name(ret_high), esp_err_to_name(ret_low));
        }

        i2c_master_bus_rm_device(dev);
    }

    i2c_del_master_bus(bus);
}

static esp_err_t camera_video_init(void)
{
    ESP_LOGI(TAG, "camera config: I2C port=%d SDA=%d SCL=%d freq=%d",
             CONFIG_OV5647_SCCB_I2C_PORT,
             CONFIG_OV5647_SCCB_SDA_GPIO,
             CONFIG_OV5647_SCCB_SCL_GPIO,
             CONFIG_OV5647_SCCB_FREQ_HZ);

    ESP_LOGI(TAG, "camera config: RESET=%d PWDN=%d LDO=esp_video/internal forced",
             CONFIG_OV5647_RESET_GPIO, CONFIG_OV5647_PWDN_GPIO);

    static const esp_video_init_csi_config_t csi_cfg = {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port = CONFIG_OV5647_SCCB_I2C_PORT,
                .scl_pin = CONFIG_OV5647_SCCB_SCL_GPIO,
                .sda_pin = CONFIG_OV5647_SCCB_SDA_GPIO,
            },
            .freq = CONFIG_OV5647_SCCB_FREQ_HZ,
        },
        .reset_pin = CONFIG_OV5647_RESET_GPIO,
        .pwdn_pin = CONFIG_OV5647_PWDN_GPIO,
        .dont_init_ldo = false,
    };

        static const esp_video_init_config_t video_cfg = {
            .csi = &csi_cfg,
        };

        esp_err_t ret = esp_video_init_with_flags(&video_cfg,
                                                  ESP_VIDEO_INIT_FLAGS_MIPI_CSI |
                                                  ESP_VIDEO_INIT_FLAGS_ISP);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "esp_video_init_with_flags failed: %s", esp_err_to_name(ret));
            return ret;
        }

        ESP_LOGI(TAG, "esp_video_init_with_flags returned OK");

        /* 二次确认 /dev/video0 是否真的生成 */
        int fd = open("/dev/video0", O_RDONLY);
        if (fd < 0) {
            ESP_LOGE(TAG, "/dev/video0 not created after esp_video_init, errno=%d", errno);
            ESP_LOGE(TAG, "Camera sensor probably not detected. Check I2C scan and OV5647 init.");
            return ESP_FAIL;
        }

        close(fd);

        ESP_LOGI(TAG, "esp_video initialized for OV5647 MIPI-CSI, /dev/video0 exists");
        return ESP_OK;
    }

static esp_err_t camera_set_format(int fd, struct v4l2_format *active_fmt)
{
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (ioctl(fd, VIDIOC_G_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "VIDIOC_G_FMT failed, errno=%d", errno);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "driver default camera format: %ux%u pixelformat=" V4L2_FMT_STR " bytesperline=%u sizeimage=%u",
             (unsigned)fmt.fmt.pix.width, (unsigned)fmt.fmt.pix.height,
             V4L2_FMT_STR_ARG(fmt.fmt.pix.pixelformat),
             (unsigned)fmt.fmt.pix.bytesperline, (unsigned)fmt.fmt.pix.sizeimage);

    const uint32_t target_w = CONFIG_CAM_H_RES;
    const uint32_t target_h = CONFIG_CAM_V_RES;
    const uint32_t source_w = CONFIG_CAM_SRC_H_RES;
    const uint32_t source_h = CONFIG_CAM_SRC_V_RES;

    /* Try to set the sensor source resolution. On this chip revision ISP cropping is
       not available, but VIDIOC_S_FMT can still select a matching sensor mode. If the
       requested source size is not available the driver will keep its default. */
    {
        struct v4l2_format set_fmt = fmt;
        set_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
        set_fmt.fmt.pix.width = source_w;
        set_fmt.fmt.pix.height = source_h;
        if (ioctl(fd, VIDIOC_S_FMT, &set_fmt) != 0) {
            ESP_LOGW(TAG, "VIDIOC_S_FMT %dx%d RGB565 failed, errno=%d; using driver default",
                     (unsigned)source_w, (unsigned)source_h, errno);
        }
        if (ioctl(fd, VIDIOC_G_FMT, &fmt) != 0) {
            ESP_LOGE(TAG, "VIDIOC_G_FMT after S_FMT failed, errno=%d", errno);
            return ESP_FAIL;
        }
    }

    if (fmt.fmt.pix.width != target_w || fmt.fmt.pix.height != target_h) {
        ESP_LOGW(TAG, "sensor provides %dx%d, target output is %dx%d; "
                 "software crop/scale will be applied in JPEG encoder if needed",
                 (unsigned)fmt.fmt.pix.width, (unsigned)fmt.fmt.pix.height,
                 target_w, target_h);
    } else {
        ESP_LOGI(TAG, "sensor output matches target %dx%d", target_w, target_h);
    }

    ESP_LOGI(TAG, "active camera format: %ux%u pixelformat=" V4L2_FMT_STR " bytesperline=%u sizeimage=%u",
             (unsigned)fmt.fmt.pix.width, (unsigned)fmt.fmt.pix.height,
             V4L2_FMT_STR_ARG(fmt.fmt.pix.pixelformat),
             (unsigned)fmt.fmt.pix.bytesperline, (unsigned)fmt.fmt.pix.sizeimage);

    s_latest_frame_width = fmt.fmt.pix.width;
    s_latest_frame_height = fmt.fmt.pix.height;

    if (fmt.fmt.pix.bytesperline == 0 && fmt.fmt.pix.width != 0) {
        fmt.fmt.pix.bytesperline = fmt.fmt.pix.width * 2;
        ESP_LOGW(TAG, "driver reported bytesperline=0, using RGB565 stride=%u",
                 (unsigned)fmt.fmt.pix.bytesperline);
    }
    if (fmt.fmt.pix.sizeimage == 0 && fmt.fmt.pix.bytesperline != 0 && fmt.fmt.pix.height != 0) {
        fmt.fmt.pix.sizeimage = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
        ESP_LOGW(TAG, "driver reported sizeimage=0, using sizeimage=%u",
                 (unsigned)fmt.fmt.pix.sizeimage);
    }

    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB565) {
        ESP_LOGW(TAG, "web preview needs RGB565, but driver returned " V4L2_FMT_STR,
                 V4L2_FMT_STR_ARG(fmt.fmt.pix.pixelformat));
    }

    if (active_fmt) {
        *active_fmt = fmt;
    }

    return ESP_OK;
}
static esp_err_t camera_start_stream(int fd, camera_buffer_t *buffers, int count)
{
    struct v4l2_requestbuffers req = {
        .count = count,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };

    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "VIDIOC_REQBUFS failed, errno=%d", errno);
        return ESP_FAIL;
    }
    if (req.count < count) {
        ESP_LOGW(TAG, "requested %d buffers, driver gave %u", count, (unsigned)req.count);
    }

    for (int i = 0; i < count; i++) {
        struct v4l2_buffer buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QUERYBUF[%d] failed, errno=%d", i, errno);
            return ESP_FAIL;
        }

        buffers[i].length = buf.length;
        buffers[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if (buffers[i].start == MAP_FAILED || buffers[i].start == NULL) {
            ESP_LOGE(TAG, "mmap[%d] failed, errno=%d", i, errno);
            return ESP_FAIL;
        }

        if (ioctl(fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "initial VIDIOC_QBUF[%d] failed, errno=%d", i, errno);
            return ESP_FAIL;
        }
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "VIDIOC_STREAMON failed, errno=%d", errno);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void camera_stop_stream_and_release(int fd, camera_buffer_t *buffers, int count)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMOFF, &type);

    for (int i = 0; i < count; i++) {
        if (buffers[i].start && buffers[i].start != MAP_FAILED) {
            munmap(buffers[i].start, buffers[i].length);
            buffers[i].start = NULL;
            buffers[i].length = 0;
        }
    }
}

static int camera_open_capture_device(const char **opened_name)
{
    static const char *device_names[] = {
        "/dev/video0",
        "/dev/video1",
        "/dev/video2",
        "/dev/video3",
        ESP_VIDEO_MIPI_CSI_DEVICE_NAME,
    };
    const int device_count = (int)(sizeof(device_names) / sizeof(device_names[0]));

    for (int n = 0; n < device_count; n++) {
        int i = (s_next_camera_device + n) % device_count;
        bool duplicate = false;
        for (int j = 0; j < i; j++) {
            if (strcmp(device_names[i], device_names[j]) == 0) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) {
            continue;
        }

        int fd = open(device_names[i], O_RDWR);
        if (fd >= 0) {
            if (opened_name) {
                *opened_name = device_names[i];
            }
            snprintf(s_camera_device_name, sizeof(s_camera_device_name), "%s", device_names[i]);
            ESP_LOGI(TAG, "opened camera capture device: %s", device_names[i]);
            s_next_camera_device = (i + 1) % device_count;
            return fd;
        }

        ESP_LOGW(TAG, "open camera device %s failed, errno=%d", device_names[i], errno);
    }

    return -1;
}


static void latest_frame_store(const void *data, size_t len,
                               uint32_t width, uint32_t height,
                               uint32_t pixfmt, uint32_t bytesperline)
{
    if (!data || len == 0 || !s_frame_mutex) {
        return;
    }

    if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }

    if (s_latest_frame_capacity < len) {
        uint8_t *new_buf = heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!new_buf) {
            ESP_LOGE(TAG, "failed to allocate latest frame buffer, len=%u", (unsigned)len);
            xSemaphoreGive(s_frame_mutex);
            return;
        }

        if (s_latest_frame) {
            free(s_latest_frame);
        }

        s_latest_frame = new_buf;
        s_latest_frame_capacity = len;
    }

    memcpy(s_latest_frame, data, len);
    s_latest_frame_len = len;
    s_latest_frame_width = width;
    s_latest_frame_height = height;
    s_latest_frame_pixfmt = pixfmt;
    s_latest_frame_bytesperline = bytesperline;
    s_latest_frame_id++;

    xSemaphoreGive(s_frame_mutex);
}

static esp_err_t latest_frame_snapshot(uint8_t **out_data, size_t *out_len,
                                       uint32_t *out_width, uint32_t *out_height,
                                       uint32_t *out_pixfmt, uint32_t *out_bytesperline,
                                       uint32_t *out_id)
{
    if (!out_data || !out_len || !out_width || !out_height || !out_pixfmt ||
        !out_bytesperline || !out_id) {
        return ESP_ERR_INVALID_ARG;
    }

    *out_data = NULL;
    *out_len = 0;

    if (!s_frame_mutex || xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (!s_latest_frame || s_latest_frame_len == 0) {
        xSemaphoreGive(s_frame_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t *copy = heap_caps_malloc(s_latest_frame_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!copy) {
        copy = heap_caps_malloc(s_latest_frame_len, MALLOC_CAP_8BIT);
    }
    if (!copy) {
        xSemaphoreGive(s_frame_mutex);
        return ESP_ERR_NO_MEM;
    }

    memcpy(copy, s_latest_frame, s_latest_frame_len);
    *out_data = copy;
    *out_len = s_latest_frame_len;
    *out_width = s_latest_frame_width;
    *out_height = s_latest_frame_height;
    *out_pixfmt = s_latest_frame_pixfmt;
    *out_bytesperline = s_latest_frame_bytesperline;
    *out_id = s_latest_frame_id;

    xSemaphoreGive(s_frame_mutex);
    return ESP_OK;
}

static void camera_capture_task(void *arg)
{
    ESP_LOGI(TAG, "camera_capture_task started");
    (void)arg;

restart_camera:
    camera_buffer_t buffers[CAMERA_BUF_COUNT] = {0};
    uint32_t empty_polls_this_device = 0;

    snprintf(s_camera_state, sizeof(s_camera_state), "opening video device");
    const char *camera_dev = NULL;
    int fd = camera_open_capture_device(&camera_dev);
    if (fd < 0) {
        snprintf(s_camera_state, sizeof(s_camera_state), "open failed");
        ESP_LOGE(TAG, "failed to open any camera video device. Check esp_video init and sensor power.");
        vTaskDelay(pdMS_TO_TICKS(1000));
        goto restart_camera;
    }

    struct v4l2_capability cap = {0};
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
        ESP_LOGI(TAG, "video driver=%s card=%s bus=%s", cap.driver, cap.card, cap.bus_info);
    }

    struct v4l2_format active_fmt = {0};
    snprintf(s_camera_state, sizeof(s_camera_state), "getting format on %s", camera_dev ? camera_dev : "unknown");
    if (camera_set_format(fd, &active_fmt) != ESP_OK) {
        snprintf(s_camera_state, sizeof(s_camera_state), "format failed on %s", camera_dev ? camera_dev : "unknown");
        close(fd);
        vTaskDelay(pdMS_TO_TICKS(500));
        goto restart_camera;
    }

    snprintf(s_camera_state, sizeof(s_camera_state), "starting stream on %s", camera_dev ? camera_dev : "unknown");
    ov5647_log_stream_regs("before STREAMON");
    if (camera_start_stream(fd, buffers, CAMERA_BUF_COUNT) != ESP_OK) {
        snprintf(s_camera_state, sizeof(s_camera_state), "stream start failed on %s", camera_dev ? camera_dev : "unknown");
        camera_stop_stream_and_release(fd, buffers, CAMERA_BUF_COUNT);
        close(fd);
        vTaskDelay(pdMS_TO_TICKS(500));
        goto restart_camera;
    }

    ESP_LOGI(TAG, "camera stream started on %s", camera_dev ? camera_dev : "unknown");
    if (s_camera_force_no_linesync) {
        ov5647_force_mipi_ctrl_no_linesync();
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    ov5647_log_stream_regs("after STREAMON");
    snprintf(s_camera_state, sizeof(s_camera_state), "polling frames on %s", camera_dev ? camera_dev : "unknown");

    uint32_t frame_count = 0;

    while (1) {
        struct v4l2_buffer buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };

        snprintf(s_camera_state, sizeof(s_camera_state), "calling dqbuf on %s", camera_dev ? camera_dev : "unknown");

        if (ioctl(fd, VIDIOC_DQBUF, &buf) != 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                s_camera_timeout_count++;
                empty_polls_this_device++;
                snprintf(s_camera_state, sizeof(s_camera_state), "waiting frame on %s", camera_dev ? camera_dev : "unknown");

                if ((s_camera_timeout_count % 20) == 0) {
                    ESP_LOGW(TAG, "camera timeout: no frame arrived on %s, count=%u",
                             camera_dev ? camera_dev : "unknown",
                             (unsigned)s_camera_timeout_count);
                }

                if (empty_polls_this_device >= CAMERA_EMPTY_POLL_LIMIT) {
                    ESP_LOGW(TAG, "no frames on %s after %u polls, restarting capture",
                             camera_dev ? camera_dev : "unknown",
                             (unsigned)empty_polls_this_device);
                    if (!s_camera_force_no_linesync) {
                        s_camera_force_no_linesync = true;
                        ESP_LOGW(TAG, "next capture restart will force OV5647 MIPI ctrl 0x4800=0x00");
                    }
                    break;
                }
            } else {
                s_camera_dqbuf_error_count++;
                snprintf(s_camera_state, sizeof(s_camera_state), "dqbuf errno=%d on %s", errno, camera_dev ? camera_dev : "unknown");
                ESP_LOGW(TAG, "VIDIOC_DQBUF failed, errno=%d", errno);
            }

            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        snprintf(s_camera_state, sizeof(s_camera_state), "receiving frames on %s", camera_dev ? camera_dev : "unknown");
        empty_polls_this_device = 0;

        if (s_camera_timeout_count != 0) {
            ESP_LOGI(TAG, "camera frame arrived after %u empty polls", (unsigned)s_camera_timeout_count);
            s_camera_timeout_count = 0;
        }

        if (buf.index >= CAMERA_BUF_COUNT) {
            ESP_LOGW(TAG, "DQBUF returned invalid index=%u", (unsigned)buf.index);
            break;
        }

        if (buf.bytesused == 0 || !buffers[buf.index].start) {
            ESP_LOGW(TAG, "DQBUF returned empty/invalid buffer: index=%u bytes=%u flags=0x%lx",
                     (unsigned)buf.index,
                     (unsigned)buf.bytesused,
                     (unsigned long)buf.flags);
        } else {
            frame_count++;

            latest_frame_store(buffers[buf.index].start,
                               buf.bytesused,
                               active_fmt.fmt.pix.width,
                               active_fmt.fmt.pix.height,
                               active_fmt.fmt.pix.pixelformat,
                               active_fmt.fmt.pix.bytesperline);

            if ((frame_count % 120) == 0) {
                ESP_LOGI(TAG, "received frame %u, index=%u, bytes=%u, fmt=%ux%u, stride=%u, flags=0x%lx",
                         (unsigned)frame_count,
                         (unsigned)buf.index,
                         (unsigned)buf.bytesused,
                         (unsigned)active_fmt.fmt.pix.width,
                         (unsigned)active_fmt.fmt.pix.height,
                         (unsigned)active_fmt.fmt.pix.bytesperline,
                         (unsigned long)buf.flags);
            }
        }

        if (ioctl(fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF failed, index=%u errno=%d", (unsigned)buf.index, errno);
            break;
        }
    }

    snprintf(s_camera_state, sizeof(s_camera_state), "restarting capture");
    camera_stop_stream_and_release(fd, buffers, CAMERA_BUF_COUNT);
    close(fd);
    vTaskDelay(pdMS_TO_TICKS(500));
    goto restart_camera;
}

static void put_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xff);
    p[1] = (uint8_t)((v >> 8) & 0xff);
}

static void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xff);
    p[1] = (uint8_t)((v >> 8) & 0xff);
    p[2] = (uint8_t)((v >> 16) & 0xff);
    p[3] = (uint8_t)((v >> 24) & 0xff);
}


static void latest_jpeg_store(const uint8_t *data, size_t len,
                              uint32_t width, uint32_t height,
                              uint32_t source_frame_id,
                              uint32_t encode_ms)
{
    if (!data || len == 0 || !s_jpeg_mutex) {
        return;
    }

    if (xSemaphoreTake(s_jpeg_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    if (s_latest_jpeg_capacity < len) {
        uint8_t *new_buf = heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!new_buf) {
            new_buf = heap_caps_malloc(len, MALLOC_CAP_8BIT);
        }
        if (!new_buf) {
            ESP_LOGE(TAG, "failed to allocate latest JPEG buffer, len=%u", (unsigned)len);
            xSemaphoreGive(s_jpeg_mutex);
            return;
        }

        if (s_latest_jpeg) {
            free(s_latest_jpeg);
        }

        s_latest_jpeg = new_buf;
        s_latest_jpeg_capacity = len;
    }

    memcpy(s_latest_jpeg, data, len);
    s_latest_jpeg_len = len;
    s_latest_jpeg_width = width;
    s_latest_jpeg_height = height;
    s_latest_jpeg_source_frame_id = source_frame_id;
    s_jpeg_last_size = (uint32_t)len;
    s_jpeg_last_ms = encode_ms;
    s_latest_jpeg_id++;

    xSemaphoreGive(s_jpeg_mutex);
}

static esp_err_t latest_jpeg_snapshot(uint8_t **out_data, size_t *out_len,
                                      uint32_t *out_width, uint32_t *out_height,
                                      uint32_t *out_jpeg_id, uint32_t *out_source_frame_id)
{
    if (!out_data || !out_len || !out_width || !out_height || !out_jpeg_id || !out_source_frame_id) {
        return ESP_ERR_INVALID_ARG;
    }

    *out_data = NULL;
    *out_len = 0;

    if (!s_jpeg_mutex || xSemaphoreTake(s_jpeg_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (!s_latest_jpeg || s_latest_jpeg_len == 0) {
        xSemaphoreGive(s_jpeg_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t *copy = heap_caps_malloc(s_latest_jpeg_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!copy) {
        copy = heap_caps_malloc(s_latest_jpeg_len, MALLOC_CAP_8BIT);
    }
    if (!copy) {
        xSemaphoreGive(s_jpeg_mutex);
        return ESP_ERR_NO_MEM;
    }

    memcpy(copy, s_latest_jpeg, s_latest_jpeg_len);
    *out_data = copy;
    *out_len = s_latest_jpeg_len;
    *out_width = s_latest_jpeg_width;
    *out_height = s_latest_jpeg_height;
    *out_jpeg_id = s_latest_jpeg_id;
    *out_source_frame_id = s_latest_jpeg_source_frame_id;

    xSemaphoreGive(s_jpeg_mutex);
    return ESP_OK;
}

static void jpeg_encode_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "jpeg_encode_task started");

    jpeg_encoder_handle_t encoder = NULL;
    jpeg_encode_engine_cfg_t eng_cfg = {
        .intr_priority = 0,
        .timeout_ms = 200,
    };

    esp_err_t ret = jpeg_new_encoder_engine(&eng_cfg, &encoder);
    if (ret != ESP_OK) {
        snprintf(s_jpeg_state, sizeof(s_jpeg_state), "encoder init failed");
        ESP_LOGE(TAG, "jpeg_new_encoder_engine failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    uint8_t *enc_in = NULL;
    uint8_t *enc_out = NULL;
    size_t enc_in_cap = 0;
    size_t enc_out_cap = 0;
    uint32_t last_encoded_frame_id = 0;
    int64_t last_encode_us = 0;

    while (1) {
        int64_t now_us = esp_timer_get_time();
        if (last_encode_us != 0 &&
            now_us - last_encode_us < (int64_t)JPEG_PREVIEW_PERIOD_MS * 1000) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        uint8_t *frame = NULL;
        size_t frame_len = 0;
        uint32_t width = 0;
        uint32_t height = 0;
        uint32_t pixfmt = 0;
        uint32_t bytesperline = 0;
        uint32_t frame_id = 0;

        ret = latest_frame_snapshot(&frame, &frame_len, &width, &height,
                                    &pixfmt, &bytesperline, &frame_id);
        if (ret != ESP_OK) {
            snprintf(s_jpeg_state, sizeof(s_jpeg_state), "waiting raw frame");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (frame_id == last_encoded_frame_id) {
            free(frame);
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        if (pixfmt != V4L2_PIX_FMT_RGB565 || width == 0 || height == 0) {
            snprintf(s_jpeg_state, sizeof(s_jpeg_state), "bad raw format");
            ESP_LOGW(TAG, "JPEG skip: pixfmt=" V4L2_FMT_STR " size=%ux%u",
                     V4L2_FMT_STR_ARG(pixfmt), (unsigned)width, (unsigned)height);
            free(frame);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (bytesperline == 0) {
            bytesperline = width * 2;
        }

        size_t packed_len = (size_t)width * (size_t)height * 2;
        size_t need_len = (size_t)bytesperline * (size_t)height;
        if (frame_len < need_len) {
            snprintf(s_jpeg_state, sizeof(s_jpeg_state), "short raw frame");
            ESP_LOGW(TAG, "JPEG skip: raw frame too small len=%u need=%u",
                     (unsigned)frame_len, (unsigned)need_len);
            free(frame);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (enc_in_cap < packed_len) {
            if (enc_in) {
                free(enc_in);
                enc_in = NULL;
                enc_in_cap = 0;
            }

            jpeg_encode_memory_alloc_cfg_t in_mem_cfg = {
                .buffer_direction = JPEG_ENC_ALLOC_INPUT_BUFFER,
            };
            size_t allocated = 0;
            enc_in = jpeg_alloc_encoder_mem(packed_len, &in_mem_cfg, &allocated);
            if (!enc_in) {
                snprintf(s_jpeg_state, sizeof(s_jpeg_state), "no encoder input mem");
                ESP_LOGE(TAG, "jpeg input alloc failed, len=%u", (unsigned)packed_len);
                free(frame);
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
            enc_in_cap = allocated;
        }

        if (enc_out_cap < packed_len) {
            if (enc_out) {
                free(enc_out);
                enc_out = NULL;
                enc_out_cap = 0;
            }

            jpeg_encode_memory_alloc_cfg_t out_mem_cfg = {
                .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
            };
            size_t allocated = 0;
            enc_out = jpeg_alloc_encoder_mem(packed_len, &out_mem_cfg, &allocated);
            if (!enc_out) {
                snprintf(s_jpeg_state, sizeof(s_jpeg_state), "no encoder output mem");
                ESP_LOGE(TAG, "jpeg output alloc failed, len=%u", (unsigned)packed_len);
                free(frame);
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
            enc_out_cap = allocated;
        }

        uint32_t out_w = CONFIG_CAM_H_RES;
        uint32_t out_h = CONFIG_CAM_V_RES;

        /* If the sensor could not be configured to the desired source size,
           clamp the output instead of failing. */
        if (out_w > width) {
            out_w = width;
        }
        if (out_h > height) {
            out_h = height;
        }

        size_t out_packed_len = (size_t)out_w * (size_t)out_h * 2;
        const uint8_t *jpeg_in = enc_in;

        if (width == out_w && height == out_h) {
            if (bytesperline == width * 2) {
                memcpy(enc_in, frame, packed_len);
            } else {
                for (uint32_t y = 0; y < height; y++) {
                    memcpy(enc_in + ((size_t)y * width * 2),
                           frame + ((size_t)y * bytesperline),
                           (size_t)width * 2);
                }
            }
        } else {
            uint32_t crop_left = (width - out_w) / 2;
            uint32_t crop_top;
            const char *anchor_name = "center";

#if CONFIG_CAM_CROP_ANCHOR_BOTTOM
            crop_top = height - out_h;
            anchor_name = "bottom";
#elif CONFIG_CAM_CROP_ANCHOR_TOP
            crop_top = 0;
            anchor_name = "top";
#else
            /* Default: center crop for a balanced far/ground view. */
            crop_top = (height - out_h) / 2;
            anchor_name = "center";
#endif

            /* If the source is contiguous (bytesperline == width*2) and we only crop
               vertically, feed the encoder directly from the selected rows to avoid memcpy. */
            if (crop_left == 0 && bytesperline == width * 2) {
                jpeg_in = frame + ((size_t)crop_top * bytesperline);
            } else {
                for (uint32_t y = 0; y < out_h; y++) {
                    memcpy(enc_in + ((size_t)y * out_w * 2),
                           frame + ((size_t)(crop_top + y) * bytesperline) + ((size_t)crop_left * 2),
                           (size_t)out_w * 2);
                }
            }
            ESP_LOGI(TAG, "software crop %ux%u -> %ux%u (%s)",
                     (unsigned)width, (unsigned)height, (unsigned)out_w, (unsigned)out_h, anchor_name);
        }

        jpeg_encode_cfg_t enc_cfg = {
            .height = out_h,
            .width = out_w,
            .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
            .sub_sample = JPEG_DOWN_SAMPLING_YUV420,
            .image_quality = JPEG_PREVIEW_QUALITY,
        };

        uint32_t jpg_size = 0;
        int64_t t0 = esp_timer_get_time();
        ret = jpeg_encoder_process(encoder, &enc_cfg,
                                   jpeg_in, (uint32_t)out_packed_len,
                                   enc_out, (uint32_t)enc_out_cap,
                                   &jpg_size);
        int64_t t1 = esp_timer_get_time();

        free(frame);

        if (ret != ESP_OK || jpg_size == 0 || jpg_size > enc_out_cap) {
            s_jpeg_encode_error_count++;
            snprintf(s_jpeg_state, sizeof(s_jpeg_state), "encode failed");
            ESP_LOGW(TAG, "jpeg_encoder_process failed: %s jpg_size=%u out_cap=%u",
                     esp_err_to_name(ret), (unsigned)jpg_size, (unsigned)enc_out_cap);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        uint32_t encode_ms = (uint32_t)((t1 - t0) / 1000);
        latest_jpeg_store(enc_out, jpg_size, out_w, out_h, frame_id, encode_ms);
        s_jpeg_encode_ok_count++;
        last_encoded_frame_id = frame_id;
        last_encode_us = t1;
        snprintf(s_jpeg_state, sizeof(s_jpeg_state), "jpeg ok");

        if ((s_jpeg_encode_ok_count % 120) == 0) {
            ESP_LOGI(TAG, "JPEG frames=%u source_frame=%u size=%u encode_ms=%u",
                     (unsigned)s_jpeg_encode_ok_count,
                     (unsigned)frame_id,
                     (unsigned)jpg_size,
                     (unsigned)encode_ms);
        }
    }
}

static esp_err_t snapshot_jpg_handler(httpd_req_t *req)
{
    HTTP_REQ_ENTER();
    uint8_t *jpg = NULL;
    size_t jpg_len = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t jpeg_id = 0;
    uint32_t source_frame_id = 0;

    esp_err_t ret = latest_jpeg_snapshot(&jpg, &jpg_len, &width, &height, &jpeg_id, &source_frame_id);
    if (ret == ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no jpeg yet");
    }
    if (ret != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "jpeg snapshot failed");
    }

    char id_str[16];
    char src_id_str[16];
    char width_str[16];
    char height_str[16];
    snprintf(id_str, sizeof(id_str), "%u", (unsigned)jpeg_id);
    snprintf(src_id_str, sizeof(src_id_str), "%u", (unsigned)source_frame_id);
    snprintf(width_str, sizeof(width_str), "%u", (unsigned)width);
    snprintf(height_str, sizeof(height_str), "%u", (unsigned)height);

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_hdr(req, "X-Jpeg-Id", id_str);
    httpd_resp_set_hdr(req, "X-Source-Frame-Id", src_id_str);
    httpd_resp_set_hdr(req, "X-Frame-Width", width_str);
    httpd_resp_set_hdr(req, "X-Frame-Height", height_str);

    ret = httpd_resp_send(req, (const char *)jpg, jpg_len);
    free(jpg);
    return ret;
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    /* Run the long-lived MJPEG stream on an async worker so the main httpd
     * task remains free to service /snapshot.jpg, /play and /imu. */
    httpd_req_t *async_req = NULL;
    esp_err_t ret = httpd_req_async_handler_begin(req, &async_req);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "stream async begin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    httpd_resp_set_type(async_req, "multipart/x-mixed-replace; boundary=" MJPEG_BOUNDARY);
    httpd_resp_set_hdr(async_req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(async_req, "Pragma", "no-cache");
    httpd_resp_set_hdr(async_req, "Access-Control-Allow-Origin", "*");

    uint32_t last_sent_jpeg_id = 0;
    char part_header[192];

    while (1) {
        uint8_t *jpg = NULL;
        size_t jpg_len = 0;
        uint32_t width = 0;
        uint32_t height = 0;
        uint32_t jpeg_id = 0;
        uint32_t source_frame_id = 0;

        ret = latest_jpeg_snapshot(&jpg, &jpg_len, &width, &height,
                                   &jpeg_id, &source_frame_id);
        if (ret != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (jpeg_id == last_sent_jpeg_id) {
            free(jpg);
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        last_sent_jpeg_id = jpeg_id;

        int header_len = snprintf(part_header, sizeof(part_header),
                                  "--" MJPEG_BOUNDARY "\r\n"
                                  "Content-Type: image/jpeg\r\n"
                                  "Content-Length: %u\r\n"
                                  "X-Jpeg-Id: %u\r\n"
                                  "X-Source-Frame-Id: %u\r\n"
                                  "X-Frame-Width: %u\r\n"
                                  "X-Frame-Height: %u\r\n"
                                  "\r\n",
                                  (unsigned)jpg_len,
                                  (unsigned)jpeg_id,
                                  (unsigned)source_frame_id,
                                  (unsigned)width,
                                  (unsigned)height);

        if (header_len <= 0 || header_len >= (int)sizeof(part_header)) {
            free(jpg);
            httpd_req_async_handler_complete(async_req);
            return ESP_FAIL;
        }

        ret = httpd_resp_send_chunk(async_req, part_header, header_len);
        if (ret != ESP_OK) {
            free(jpg);
            break;
        }

        ret = httpd_resp_send_chunk(async_req, (const char *)jpg, jpg_len);
        free(jpg);
        if (ret != ESP_OK) {
            break;
        }

        ret = httpd_resp_send_chunk(async_req, "\r\n", 2);
        if (ret != ESP_OK) {
            break;
        }
    }

    httpd_req_async_handler_complete(async_req);
    return ESP_OK;
}




static esp_err_t index_handler(httpd_req_t *req)
{
    static const char html[] =
        "<!doctype html><html><head><meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<title>ESP32-P4 Guide Glasses</title>"
        "<style>"
        "body{margin:0;background:#101418;color:#eef3f7;font-family:Arial,sans-serif;text-align:center;padding:24px;}"
        "a{color:#9bd1ff;}"
        "</style></head><body>"
        "<h2>ESP32-P4 导盲眼镜服务端</h2>"
        "<p>请使用 PC Agent 监控网页查看实时视频流。</p>"
        "<p><a href=\"/snapshot.jpg\" target=\"_blank\">snapshot.jpg</a> | "
        "<a href=\"/status\" target=\"_blank\">status</a> | "
        "<a href=\"/imu\" target=\"_blank\">imu.json</a></p>"
        "</body></html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t frame_bmp_handler(httpd_req_t *req)
{
    uint8_t *frame = NULL;
    size_t frame_len = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pixfmt = 0;
    uint32_t bytesperline = 0;
    uint32_t frame_id = 0;

    esp_err_t ret = latest_frame_snapshot(&frame, &frame_len, &width, &height,
                                          &pixfmt, &bytesperline, &frame_id);
    if (ret == ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no frame yet");
    }
    if (ret != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "frame snapshot failed");
    }

    if (pixfmt != V4L2_PIX_FMT_RGB565) {
        free(frame);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "camera frame is not RGB565");
    }

    if (bytesperline == 0) {
        bytesperline = width * 2;
    }

    size_t need = (size_t)bytesperline * (size_t)height;
    if (width == 0 || height == 0 || frame_len < need) {
        free(frame);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "invalid RGB565 frame");
    }

    const uint32_t row_stride = (uint32_t)(((width * 3) + 3) & ~3U);
    const uint32_t image_size = row_stride * height;
    const uint32_t file_size = 54 + image_size;
    uint8_t header[54] = {0};

    header[0] = 'B';
    header[1] = 'M';
    put_le32(&header[2], file_size);
    put_le32(&header[10], 54);
    put_le32(&header[14], 40);
    put_le32(&header[18], width);
    put_le32(&header[22], height);
    put_le16(&header[26], 1);
    put_le16(&header[28], 24);
    put_le32(&header[34], image_size);

    uint8_t *row = heap_caps_malloc(row_stride, MALLOC_CAP_8BIT);
    if (!row) {
        free(frame);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "row allocation failed");
    }

    char cache_hdr[48];
    snprintf(cache_hdr, sizeof(cache_hdr), "no-store, frame=%u", (unsigned)frame_id);
    httpd_resp_set_type(req, "image/bmp");
    httpd_resp_set_hdr(req, "Cache-Control", cache_hdr);
    httpd_resp_set_hdr(req, "Connection", "close");
    ESP_GOTO_ON_ERROR(httpd_resp_send_chunk(req, (const char *)header, sizeof(header)),
                      cleanup, TAG, "send bmp header failed");

    for (int y = (int)height - 1; y >= 0; y--) {
        const uint8_t *src = frame + ((size_t)y * bytesperline);
        memset(row, 0, row_stride);
        for (uint32_t x = 0; x < width; x++) {
            uint16_t p = (uint16_t)src[x * 2] | ((uint16_t)src[x * 2 + 1] << 8);
            uint8_t r = (uint8_t)((p >> 11) & 0x1f);
            uint8_t g = (uint8_t)((p >> 5) & 0x3f);
            uint8_t b = (uint8_t)(p & 0x1f);
            row[x * 3 + 0] = (uint8_t)((b << 3) | (b >> 2));
            row[x * 3 + 1] = (uint8_t)((g << 2) | (g >> 4));
            row[x * 3 + 2] = (uint8_t)((r << 3) | (r >> 2));
        }
        ESP_GOTO_ON_ERROR(httpd_resp_send_chunk(req, (const char *)row, row_stride),
                          cleanup, TAG, "send bmp row failed");
    }

    ret = httpd_resp_send_chunk(req, NULL, 0);

cleanup:
    free(row);
    free(frame);
    return ret;
}

static esp_err_t frame_raw_handler(httpd_req_t *req)
{
    uint8_t *frame_copy = NULL;
    size_t frame_len = 0;
    uint32_t frame_width = 0;
    uint32_t frame_height = 0;

    if (!s_frame_mutex || xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "frame mutex not ready");
    }

    if (!s_latest_frame || s_latest_frame_len == 0) {
        xSemaphoreGive(s_frame_mutex);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no frame yet");
    }

    frame_len = s_latest_frame_len;
    frame_width = s_latest_frame_width;
    frame_height = s_latest_frame_height;

    frame_copy = heap_caps_malloc(frame_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!frame_copy) {
        xSemaphoreGive(s_frame_mutex);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory for frame copy");
    }

    memcpy(frame_copy, s_latest_frame, frame_len);
    xSemaphoreGive(s_frame_mutex);

    char width_str[16];
    char height_str[16];
    char len_str[24];

    snprintf(width_str, sizeof(width_str), "%u", (unsigned)frame_width);
    snprintf(height_str, sizeof(height_str), "%u", (unsigned)frame_height);
    snprintf(len_str, sizeof(len_str), "%u", (unsigned)frame_len);

    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_hdr(req, "X-Frame-Width", width_str);
    httpd_resp_set_hdr(req, "X-Frame-Height", height_str);
    httpd_resp_set_hdr(req, "X-Frame-Len", len_str);
    httpd_resp_set_hdr(req, "X-Frame-Format", "RGB565");

    esp_err_t ret = httpd_resp_send(req, (const char *)frame_copy, frame_len);

    free(frame_copy);
    return ret;
}





static esp_err_t status_handler(httpd_req_t *req)
{
    HTTP_REQ_ENTER();
    char body[1280];

    uint32_t frame_id = s_latest_frame_id;
    uint32_t width = s_latest_frame_width;
    uint32_t height = s_latest_frame_height;
    uint32_t pixfmt = s_latest_frame_pixfmt;
    uint32_t stride = s_latest_frame_bytesperline;
    size_t len = s_latest_frame_len;

    imu_attitude_t att = {0};
    imu_stats_t imu_stats = {0};
    bool imu_ok = (imu_atk_ms601m_get_attitude(&att) == ESP_OK) &&
                  (imu_atk_ms601m_get_stats(&imu_stats) == ESP_OK);

    bool turn_in_progress = false;
    turn_dir_t turn_dir = TURN_DIR_NONE;
    float turn_progress_deg = 0.0f;
    float last_completed_turn_deg = 0.0f;
    uint32_t turn_completed_count = 0;
    char turn_state_str[32] = "idle";
    if (s_turn_mutex && xSemaphoreTake(s_turn_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        turn_in_progress = s_turn_in_progress;
        turn_dir = s_turn_dir;
        turn_progress_deg = s_turn_progress_deg;
        last_completed_turn_deg = s_last_completed_turn_deg;
        turn_completed_count = s_turn_completed_count;
        snprintf(turn_state_str, sizeof(turn_state_str), "%s", s_turn_state_str);
        xSemaphoreGive(s_turn_mutex);
    }

    bool sd_mounted = sd_card_is_mounted();
    uint64_t sd_free_bytes = 0;
    uint64_t sd_total_bytes = 0;
    if (sd_mounted) {
        sd_card_get_free_space(&sd_free_bytes, &sd_total_bytes);
    }

    snprintf(body, sizeof(body),
             "camera_state=%s\n"
             "camera_device=%s\n"
             "raw_frames=%u\n"
             "raw_last_len=%u\n"
             "raw_size=%ux%u\n"
             "raw_stride=%u\n"
             "raw_format=" V4L2_FMT_STR "\n"
             "camera_timeouts=%u\n"
             "camera_dqbuf_errors=%u\n"
             "jpeg_state=%s\n"
             "jpeg_frames=%u\n"
             "jpeg_source_frame=%u\n"
             "jpeg_size=%u\n"
             "jpeg_last_encode_ms=%u\n"
             "jpeg_errors=%u\n"
             "jpeg_out_size=%ux%u\n"
             "preview=/stream\n"
             "imu_ready=%s\n"
             "imu_roll=%.2f\n"
             "imu_pitch=%.2f\n"
             "imu_yaw=%.2f\n"
             "imu_frames=%u\n"
             "imu_bytes=%u\n"
             "imu_errors=%u\n"
             "imu_url=/imu\n"
             "turn_state=%s\n"
             "turn_in_progress=%s\n"
             "turn_dir=%s\n"
             "turn_progress_deg=%.1f\n"
             "turn_completed_count=%u\n"
             "turn_last_completed_deg=%.1f\n"
             "sd_mounted=%s\n"
             "sd_free_mb=%.1f\n"
             "sd_total_mb=%.1f\n"
             "sd_photo_dir=%s\n",
             s_camera_state,
             s_camera_device_name,
             (unsigned)frame_id,
             (unsigned)len,
             (unsigned)width,
             (unsigned)height,
             (unsigned)stride,
             V4L2_FMT_STR_ARG(pixfmt),
             (unsigned)s_camera_timeout_count,
             (unsigned)s_camera_dqbuf_error_count,
             s_jpeg_state,
             (unsigned)s_latest_jpeg_id,
             (unsigned)s_latest_jpeg_source_frame_id,
             (unsigned)s_jpeg_last_size,
             (unsigned)s_jpeg_last_ms,
             (unsigned)s_jpeg_encode_error_count,
             (unsigned)CONFIG_CAM_H_RES,
             (unsigned)CONFIG_CAM_V_RES,
             imu_ok ? "yes" : "no",
             att.roll, att.pitch, att.yaw,
             (unsigned)imu_stats.frames_parsed,
             (unsigned)imu_stats.bytes_received,
             (unsigned)imu_stats.checksum_errors,
             turn_state_str,
             turn_in_progress ? "yes" : "no",
             turn_dir_name(turn_dir),
             turn_progress_deg,
             (unsigned)turn_completed_count,
             last_completed_turn_deg,
             sd_mounted ? "yes" : "no",
             (double)sd_free_bytes / (1024.0 * 1024.0),
             (double)sd_total_bytes / (1024.0 * 1024.0),
             sd_card_get_photo_dir() ? sd_card_get_photo_dir() : "SD disabled");

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t imu_handler(httpd_req_t *req)
{
    imu_attitude_t att = {0};
    imu_motion_t motion = {0};
    imu_stats_t stats = {0};

    esp_err_t ret_att = imu_atk_ms601m_get_attitude(&att);
    (void)imu_atk_ms601m_get_motion(&motion);
    (void)imu_atk_ms601m_get_stats(&stats);

    bool turn_in_progress = false;
    turn_dir_t turn_dir = TURN_DIR_NONE;
    float turn_progress_deg = 0.0f;
    float last_completed_turn_deg = 0.0f;
    uint32_t turn_completed_count = 0;
    char turn_state_str[32] = "unknown";
    if (s_turn_mutex && xSemaphoreTake(s_turn_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        turn_in_progress = s_turn_in_progress;
        turn_dir = s_turn_dir;
        turn_progress_deg = s_turn_progress_deg;
        last_completed_turn_deg = s_last_completed_turn_deg;
        turn_completed_count = s_turn_completed_count;
        snprintf(turn_state_str, sizeof(turn_state_str), "%s", s_turn_state_str);
        xSemaphoreGive(s_turn_mutex);
    }

    char body[1024];
    if (ret_att == ESP_OK) {
        snprintf(body, sizeof(body),
                 "{\n"
                 "  \"attitude\": {\n"
                 "    \"roll\": %.2f,\n"
                 "    \"pitch\": %.2f,\n"
                 "    \"yaw\": %.2f\n"
                 "  },\n"
                 "  \"motion\": {\n"
                 "    \"acc_g\": [%.3f, %.3f, %.3f],\n"
                 "    \"gyro_dps\": [%.2f, %.2f, %.2f]\n"
                 "  },\n"
                 "  \"turn\": {\n"
                 "    \"state\": \"%s\",\n"
                 "    \"in_progress\": %s,\n"
                 "    \"dir\": \"%s\",\n"
                 "    \"progress_deg\": %.1f,\n"
                 "    \"completed_count\": %u,\n"
                 "    \"last_completed_deg\": %.1f\n"
                 "  },\n"
                 "  \"stats\": {\n"
                 "    \"frames_parsed\": %u,\n"
                 "    \"bytes_received\": %u,\n"
                 "    \"checksum_errors\": %u\n"
                 "  }\n"
                 "}\n",
                 att.roll, att.pitch, att.yaw,
                 motion.acc_g[0], motion.acc_g[1], motion.acc_g[2],
                 motion.gyro_dps[0], motion.gyro_dps[1], motion.gyro_dps[2],
                 turn_state_str,
                 turn_in_progress ? "true" : "false",
                 turn_dir_name(turn_dir),
                 turn_progress_deg,
                 (unsigned)turn_completed_count,
                 last_completed_turn_deg,
                 (unsigned)stats.frames_parsed,
                 (unsigned)stats.bytes_received,
                 (unsigned)stats.checksum_errors);
    } else {
        snprintf(body, sizeof(body),
                 "{\"error\":\"IMU not ready\",\"code\":\"%s\"}\n",
                 esp_err_to_name(ret_att));
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t imu_html_handler(httpd_req_t *req)
{
    HTTP_REQ_ENTER();
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, IMU_DASHBOARD_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "capture: handler called");

    if (!sd_card_is_mounted()) {
        ESP_LOGW(TAG, "capture: SD not mounted, attempting on-demand mount");
        if (sd_card_init() != ESP_OK || !sd_card_is_mounted()) {
            ESP_LOGW(TAG, "capture: SD mount failed, returning error");
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                       "SD card not ready");
        }
    }

    uint8_t *jpg = NULL;
    size_t jpg_len = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t jpeg_id = 0;
    uint32_t source_frame_id = 0;

    esp_err_t ret = latest_jpeg_snapshot(&jpg, &jpg_len, &width, &height,
                                         &jpeg_id, &source_frame_id);
    if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "capture: no jpeg available yet");
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "no jpeg yet");
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "capture: jpeg snapshot failed: %s", esp_err_to_name(ret));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "jpeg snapshot failed");
    }

    ESP_LOGI(TAG, "capture: got jpeg %u bytes (%ux%u)",
             (unsigned)jpg_len, (unsigned)width, (unsigned)height);

    /* Generate the target path up-front so the browser gets the filename
     * immediately. The actual SD write happens after the HTTP response is
     * sent, so the UI never appears to hang even if the card write is slow. */
    char saved_path[128] = {0};
    bool sd_ready = sd_card_is_mounted();
    if (sd_ready) {
        sd_card_generate_photo_path(saved_path, sizeof(saved_path));
        ESP_LOGI(TAG, "capture: target path %s", saved_path);
    }

    char id_str[16];
    char src_id_str[16];
    char width_str[16];
    char height_str[16];
    snprintf(id_str, sizeof(id_str), "%u", (unsigned)jpeg_id);
    snprintf(src_id_str, sizeof(src_id_str), "%u", (unsigned)source_frame_id);
    snprintf(width_str, sizeof(width_str), "%u", (unsigned)width);
    snprintf(height_str, sizeof(height_str), "%u", (unsigned)height);

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_hdr(req, "X-Jpeg-Id", id_str);
    httpd_resp_set_hdr(req, "X-Source-Frame-Id", src_id_str);
    httpd_resp_set_hdr(req, "X-Frame-Width", width_str);
    httpd_resp_set_hdr(req, "X-Frame-Height", height_str);
    httpd_resp_set_hdr(req, "X-SD-Saved", sd_ready ? "pending" : "0");
    if (sd_ready && saved_path[0]) {
        httpd_resp_set_hdr(req, "X-SD-Path", saved_path);
    }

    /* Send the image back to the browser first. */
    ret = httpd_resp_send(req, (const char *)jpg, jpg_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "capture: failed to send response: %s", esp_err_to_name(ret));
        free(jpg);
        return ret;
    }

    /* Save to SD after the response has been sent. This blocks only this
     * HTTP session task, not the browser. */
    if (sd_ready && saved_path[0]) {
        ESP_LOGI(TAG, "capture: writing %u bytes to %s...",
                 (unsigned)jpg_len, saved_path);
        int64_t t0 = esp_timer_get_time();
        esp_err_t save_ret = sd_card_save_photo_to_path(jpg, jpg_len, saved_path);
        int64_t t1 = esp_timer_get_time();
        if (save_ret == ESP_OK) {
            ESP_LOGI(TAG, "capture: saved in %lld ms", (t1 - t0) / 1000);
        } else {
            ESP_LOGW(TAG, "capture: save failed: %s (%lld ms)",
                     esp_err_to_name(save_ret), (t1 - t0) / 1000);
        }
    } else {
        ESP_LOGW(TAG, "capture: SD not mounted, photo not saved");
    }

    free(jpg);
    return ESP_OK;
}

static esp_err_t imu_stream_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/event-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");

    while (1) {
        imu_attitude_t att = {0};
        imu_stats_t stats = {0};
        bool ok = (imu_atk_ms601m_get_attitude(&att) == ESP_OK) &&
                  (imu_atk_ms601m_get_stats(&stats) == ESP_OK);

        char event[256];
        int len = snprintf(event, sizeof(event),
                           "data: {\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
                           "\"frames\":%u,\"bytes\":%u,\"errors\":%u,\"ready\":%s}\n\n",
                           att.roll, att.pitch, att.yaw,
                           (unsigned)stats.frames_parsed,
                           (unsigned)stats.bytes_received,
                           (unsigned)stats.checksum_errors,
                           ok ? "true" : "false");

        if (len <= 0 || len >= (int)sizeof(event)) {
            break;
        }

        esp_err_t ret = httpd_resp_send_chunk(req, event, len);
        if (ret != ESP_OK) {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return ESP_OK;
}

static esp_err_t photos_handler(httpd_req_t *req)
{
    const char *dir = sd_card_get_photo_dir();
    if (!dir) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "SD card disabled");
    }
    DIR *d = opendir(dir);
    if (!d) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "cannot open photo dir");
    }

    size_t cap = 4096;
    size_t len = 1;
    char *body = heap_caps_malloc(cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!body) {
        body = heap_caps_malloc(cap, MALLOC_CAP_8BIT);
    }
    if (!body) {
        closedir(d);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "no memory for photo list");
    }
    body[0] = '[';

    struct dirent *entry;
    while ((entry = readdir(d)) != NULL) {
        const char *name = entry->d_name;
        if (strstr(name, ".jpg") == NULL) {
            continue;
        }

        char path[512];
        size_t dir_len = strlen(dir);
        size_t name_len = strlen(name);
        if (dir_len + 1 + name_len >= sizeof(path)) {
            continue;
        }
        memcpy(path, dir, dir_len);
        path[dir_len] = '/';
        memcpy(path + dir_len + 1, name, name_len + 1);

        struct stat st;
        if (stat(path, &st) != 0) {
            continue;
        }

        char item[256];
        int item_len = snprintf(item, sizeof(item),
                                "%s{\"name\":\"%s\",\"size\":%ld,\"mtime\":%ld}",
                                len > 1 ? "," : "", name,
                                (long)st.st_size, (long)st.st_mtime);
        if (item_len <= 0) {
            continue;
        }

        while (len + item_len + 2 > cap) {
            size_t new_cap = cap * 2;
            char *new_body = heap_caps_realloc(body, new_cap,
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!new_body) {
                new_body = heap_caps_realloc(body, new_cap, MALLOC_CAP_8BIT);
            }
            if (!new_body) {
                item_len = 0;
                break;
            }
            body = new_body;
            cap = new_cap;
        }
        if (item_len == 0) {
            break;
        }

        memcpy(body + len, item, item_len);
        len += item_len;
    }
    closedir(d);

    body[len++] = ']';
    body[len] = '\0';

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Connection", "close");
    esp_err_t ret = httpd_resp_send(req, body, len);
    free(body);
    return ret;
}

static esp_err_t play_handler(httpd_req_t *req)
{
    /* 读取 Content-Length */
    char content_len_str[16] = {0};
    size_t content_len = 0;
    if (httpd_req_get_hdr_value_str(req, "Content-Length", content_len_str, sizeof(content_len_str)) == ESP_OK) {
        content_len = (size_t)atoi(content_len_str);
    }

    if (content_len == 0) {
        httpd_resp_set_status(req, "411 Length Required");
        return httpd_resp_send(req, "Content-Length required", HTTPD_RESP_USE_STRLEN);
    }

    /* 限制单次音频大小：5 秒 16kHz 16bit 单声道 ≈ 160KB */
    const size_t MAX_PCM_LEN = 160 * 1024;
    if (content_len > MAX_PCM_LEN) {
        httpd_resp_set_status(req, "413 Payload Too Large");
        return httpd_resp_send(req, "PCM too large", HTTPD_RESP_USE_STRLEN);
    }

    uint8_t *pcm_buf = (uint8_t *)malloc(content_len);
    if (pcm_buf == NULL) {
        ESP_LOGE(TAG, "play_handler: failed to allocate %d bytes", (int)content_len);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory");
    }

    int received = 0;
    int remaining = (int)content_len;
    while (remaining > 0) {
        int ret = httpd_req_recv(req, (char *)pcm_buf + received, remaining);
        if (ret <= 0) {
            ESP_LOGE(TAG, "play_handler: recv failed, ret=%d", ret);
            free(pcm_buf);
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
        }
        received += ret;
        remaining -= ret;
    }

    HTTP_REQ_ENTER();
    ESP_LOGI(TAG, "play_handler: received %d bytes PCM, queueing...", received);

    audio_queue_item_t item = {
        .pcm = pcm_buf,
        .len = (size_t)received,
    };
    if (s_audio_queue == NULL) {
        free(pcm_buf);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "audio queue not ready");
    }

    if (xQueueSend(s_audio_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
        free(pcm_buf);
        ESP_LOGW(TAG, "play_handler: audio queue full");
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "audio queue full", HTTPD_RESP_USE_STRLEN);
    }

    /* Mark activity so the offline fallback tone does not fire while audio is queued. */
    s_last_play_ticks = xTaskGetTickCount();

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t volume_handler(httpd_req_t *req)
{
    HTTP_REQ_ENTER();
    int volume = 0;
    char query[32] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[8] = {0};
        if (httpd_query_key_value(query, "vol", val, sizeof(val)) == ESP_OK) {
            volume = atoi(val);
        }
    }
    if (volume < 0) volume = 0;
    if (volume > 100) volume = 100;

    esp_err_t ret = audio_speaker_set_volume(volume);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "volume_handler: set volume failed: %s", esp_err_to_name(ret));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "set volume failed");
    }
    /* Explicitly unmute when any volume is set; the codec may stay muted after init. */
    if (volume > 0) {
        audio_speaker_mute(false);
    }

    char resp[32];
    snprintf(resp, sizeof(resp), "volume=%d", volume);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t ping_handler(httpd_req_t *req)
{
    HTTP_REQ_ENTER();
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "pong", 4);
}

static esp_err_t beep_handler(httpd_req_t *req)
{
    HTTP_REQ_ENTER();
    esp_err_t ret = audio_speaker_beep_test();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "beep_handler: beep failed: %s", esp_err_to_name(ret));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "beep failed");
    }
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "beep", 4);
}

#define OFFLINE_TIMEOUT_MS    10000
#define OFFLINE_TONE_MS       700
#define OFFLINE_SAMPLE_RATE   16000
#define OFFLINE_BEEP_MS       150
#define OFFLINE_GAP_MS        100
#define OFFLINE_TONE_AMP      6000

static void generate_offline_tone(int16_t *buf, size_t samples)
{
    int beep1_samples = OFFLINE_BEEP_MS * OFFLINE_SAMPLE_RATE / 1000;
    int gap_samples = OFFLINE_GAP_MS * OFFLINE_SAMPLE_RATE / 1000;
    int beep2_start = beep1_samples + gap_samples;
    int beep2_samples = OFFLINE_BEEP_MS * OFFLINE_SAMPLE_RATE / 1000;

    for (size_t i = 0; i < samples; i++) {
        if (i < (size_t)beep1_samples) {
            buf[i] = (int16_t)(OFFLINE_TONE_AMP *
                               sinf(2.0f * 3.14159265358979323846f * 1000.0f * (float)i / OFFLINE_SAMPLE_RATE));
        } else if (i >= (size_t)beep2_start && i < (size_t)(beep2_start + beep2_samples)) {
            int j = (int)i - beep2_start;
            buf[i] = (int16_t)(OFFLINE_TONE_AMP *
                               sinf(2.0f * 3.14159265358979323846f * 800.0f * (float)j / OFFLINE_SAMPLE_RATE));
        } else {
            buf[i] = 0;
        }
    }
}

static void offline_tone_task(void *arg)
{
    (void)arg;
    s_last_play_ticks = xTaskGetTickCount();

    const size_t tone_samples = OFFLINE_TONE_MS * OFFLINE_SAMPLE_RATE / 1000;
    const size_t tone_len = tone_samples * sizeof(int16_t);
    int16_t *tone_buf = (int16_t *)malloc(tone_len);
    if (tone_buf == NULL) {
        ESP_LOGE(TAG, "offline_tone_task: failed to allocate tone buffer");
        vTaskDelete(NULL);
        return;
    }
    generate_offline_tone(tone_buf, tone_samples);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (s_speaker_mutex == NULL) {
            continue;
        }
        TickType_t now = xTaskGetTickCount();
        if ((now - s_last_play_ticks) < pdMS_TO_TICKS(OFFLINE_TIMEOUT_MS)) {
            continue;
        }
        ESP_LOGW(TAG, "offline_tone_task: no /play for %d ms, playing fallback tone", OFFLINE_TIMEOUT_MS);
        if (xSemaphoreTake(s_speaker_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
            audio_speaker_play((const uint8_t *)tone_buf, tone_len, pdMS_TO_TICKS(2000));
            xSemaphoreGive(s_speaker_mutex);
        }
        s_last_play_ticks = xTaskGetTickCount();
    }
}

static void audio_playback_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "audio_playback_task started");

    audio_queue_item_t item;
    while (1) {
        if (xQueueReceive(s_audio_queue, &item, portMAX_DELAY) == pdTRUE) {
            if (item.pcm == NULL || item.len == 0) {
                continue;
            }
            if (s_speaker_mutex == NULL ||
                xSemaphoreTake(s_speaker_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
                ESP_LOGW(TAG, "audio_playback_task: failed to take speaker mutex");
                free(item.pcm);
                continue;
            }
            s_last_play_ticks = xTaskGetTickCount();
            esp_err_t play_ret = audio_speaker_play(item.pcm, item.len, pdMS_TO_TICKS(5000));
            if (play_ret != ESP_OK) {
                ESP_LOGW(TAG, "audio_playback_task: audio_speaker_play failed: %s", esp_err_to_name(play_ret));
            }
            xSemaphoreGive(s_speaker_mutex);
            free(item.pcm);
        }
    }
}

static void http_watchdog_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "http_watchdog_task started, timeout=%d ms", HTTP_WATCHDOG_TIMEOUT_MS);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));

        if (s_http_request_count == 0) {
            /* No request has ever been served; device is idle, do not reboot. */
            continue;
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - s_last_http_request_ticks) > pdMS_TO_TICKS(HTTP_WATCHDOG_TIMEOUT_MS)) {
            ESP_LOGE(TAG, "HTTP watchdog: no request served for %d ms, rebooting", HTTP_WATCHDOG_TIMEOUT_MS);
            esp_restart();
        }
    }
}

static esp_err_t start_mdns(void)
{
    esp_err_t ret = mdns_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mdns_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    mdns_hostname_set("guide-glasses");
    mdns_instance_name_set("Guide Glasses ESP32-P4");

    ret = mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mdns_service_add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "mDNS started: http://guide-glasses.local/");
    return ESP_OK;
}

static esp_err_t start_web_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 12288;
    config.max_uri_handlers = 15;
    config.max_open_sockets = 6;
    config.lru_purge_enable = true;
    config.keep_alive_enable = false;
    config.recv_wait_timeout = 5;
    config.send_wait_timeout = 5;

    httpd_handle_t server = NULL;
    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = NULL,
    };

    /* The long-running MJPEG /stream endpoint runs on an async worker so it
     * does not block /snapshot.jpg, /play and /imu on the main httpd task. */

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t snapshot_jpg_uri = {
        .uri = "/snapshot.jpg",
        .method = HTTP_GET,
        .handler = snapshot_jpg_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t capture_uri = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = capture_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t frame_raw_uri = {
        .uri = "/frame.raw",
        .method = HTTP_GET,
        .handler = frame_raw_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t frame_bmp_uri = {
        .uri = "/frame.bmp",
        .method = HTTP_GET,
        .handler = frame_bmp_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = status_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t imu_uri = {
        .uri = "/imu",
        .method = HTTP_GET,
        .handler = imu_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t imu_html_uri = {
        .uri = "/imu.html",
        .method = HTTP_GET,
        .handler = imu_html_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t photos_uri = {
        .uri = "/photos",
        .method = HTTP_GET,
        .handler = photos_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t play_uri = {
        .uri = "/play",
        .method = HTTP_POST,
        .handler = play_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t volume_uri = {
        .uri = "/volume",
        .method = HTTP_GET,
        .handler = volume_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t ping_uri = {
        .uri = "/ping",
        .method = HTTP_GET,
        .handler = ping_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t beep_uri = {
        .uri = "/beep",
        .method = HTTP_GET,
        .handler = beep_handler,
        .user_ctx = NULL,
    };

    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &index_uri),
                        TAG, "register / failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &stream_uri),
                        TAG, "register /stream failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &snapshot_jpg_uri),
                        TAG, "register /snapshot.jpg failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &capture_uri),
                        TAG, "register /capture failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &frame_raw_uri),
                        TAG, "register /frame.raw failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &frame_bmp_uri),
                        TAG, "register /frame.bmp failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &status_uri),
                        TAG, "register /status failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &imu_uri),
                        TAG, "register /imu failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &imu_html_uri),
                        TAG, "register /imu.html failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &photos_uri),
                        TAG, "register /photos failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &play_uri),
                        TAG, "register /play failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &volume_uri),
                        TAG, "register /volume failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &ping_uri),
                        TAG, "register /ping failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &beep_uri),
                        TAG, "register /beep failed");

    ESP_LOGI(TAG, "HTTP server started: http://%s/", s_board_ip);
    ESP_LOGI(TAG, "MJPEG stream: http://%s/stream", s_board_ip);
    ESP_LOGI(TAG, "Snapshot: http://%s/snapshot.jpg", s_board_ip);
    ESP_LOGI(TAG, "Status: http://%s/status", s_board_ip);
    ESP_LOGI(TAG, "Photos: http://%s/photos", s_board_ip);
    return ESP_OK;
}

    void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /*
     * ATK-MS601M IMU on UART1 (GPIO21/22)
     */
    esp_err_t imu_ret = imu_atk_ms601m_init();
    if (imu_ret == ESP_OK && imu_atk_ms601m_is_initialized()) {
        xTaskCreatePinnedToCore(imu_log_task,
                                "imu_log",
                                4096,
                                NULL,
                                2,
                                NULL,
                                0);

        s_turn_mutex = xSemaphoreCreateMutex();
        if (s_turn_mutex != NULL) {
            xTaskCreatePinnedToCore(imu_turn_task,
                                    "imu_turn",
                                    4096,
                                    NULL,
                                    5,
                                    NULL,
                                    0);
        } else {
            ESP_LOGE(TAG, "failed to create turn mutex, turn detection disabled");
        }
    } else if (imu_ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU init failed: %s", esp_err_to_name(imu_ret));
    } else {
        ESP_LOGW(TAG, "IMU disabled in menuconfig");
    }

    /*
     * Wi-Fi first. The on-board ESP32-C6 uses the same SDMMC host as the TF
     * card (slot 1 vs slot 0). ESP-Hosted must initialise the SDIO link before
     * we try to mount the SD card, otherwise the SD card probing breaks Wi-Fi.
     */
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    /*
     * Camera (and optional speaker) after Wi-Fi is up.
     */
    esp_err_t cam_ret = camera_video_init();
    if (cam_ret != ESP_OK) {
        ESP_LOGE(TAG, "camera init failed, continue Wi-Fi only: %s", esp_err_to_name(cam_ret));
    } else {
#if CONFIG_AUDIO_SPEAKER_ENABLE
        /* Initialize speaker after camera so the shared I2C bus is ready. */
        esp_err_t spk_ret = audio_speaker_init();
        if (spk_ret == ESP_OK) {
            /* 默认音量 100%，户外录制需要足够响亮；也可通过 /volume?vol=xxx 调整 */
            audio_speaker_set_volume(100);
            audio_speaker_mute(false);

            s_speaker_mutex = xSemaphoreCreateMutex();
            if (s_speaker_mutex != NULL) {
                s_last_play_ticks = xTaskGetTickCount();

                s_audio_queue = xQueueCreate(AUDIO_QUEUE_LEN, sizeof(audio_queue_item_t));
                if (s_audio_queue != NULL) {
                    xTaskCreatePinnedToCore(audio_playback_task,
                                            "audio_playback",
                                            4096,
                                            NULL,
                                            5,
                                            NULL,
                                            0);
                } else {
                    ESP_LOGE(TAG, "failed to create audio playback queue");
                }

                xTaskCreatePinnedToCore(offline_tone_task,
                                        "offline_tone",
                                        4096,
                                        NULL,
                                        2,
                                        NULL,
                                        0);
            } else {
                ESP_LOGE(TAG, "failed to create speaker mutex, offline tone disabled");
            }
        } else {
            ESP_LOGE(TAG, "speaker init failed, continue camera only: %s", esp_err_to_name(spk_ret));
        }
#endif

        s_frame_mutex = xSemaphoreCreateMutex();
        s_jpeg_mutex = xSemaphoreCreateMutex();
        if (!s_frame_mutex || !s_jpeg_mutex) {
            ESP_LOGE(TAG, "failed to create frame/jpeg mutex");
        } else {
            xTaskCreatePinnedToCore(camera_capture_task,
                                    "camera_capture",
                                    8192,
                                    NULL,
                                    6,
                                    NULL,
                                    1);

            xTaskCreatePinnedToCore(jpeg_encode_task,
                                    "jpeg_encode",
                                    8192,
                                    NULL,
                                    3,
                                    NULL,
                                    0);
        }
    }

    /*
     * Onboard SD card (TF slot) for saving captured photos.
     * Must be initialised after ESP-Hosted/Wi-Fi has claimed SDMMC slot 1.
     */
    esp_err_t sd_ret = sd_card_init();
    if (sd_ret == ESP_OK && sd_card_is_mounted()) {
        ESP_LOGI(TAG, "SD card ready for photo capture");
    } else if (sd_ret != ESP_OK) {
        ESP_LOGW(TAG, "SD card init failed (card may not be inserted): %s",
                 esp_err_to_name(sd_ret));
    }

    /* Retry SD mount in background to support hot-plug. */
    sd_card_start_monitor_task();

    if (s_sta_netif) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(s_sta_netif, &ip_info) == ESP_OK) {
            snprintf(s_board_ip, sizeof(s_board_ip), IPSTR, IP2STR(&ip_info.ip));
            ESP_LOGI(TAG, "Final Camera web URL: http://%s/", s_board_ip);
            ESP_LOGI(TAG, "Final MJPEG stream URL: http://%s/stream", s_board_ip);
        }
    }

    if (s_sta_netif) {
        ESP_ERROR_CHECK(start_web_server());
        start_mdns();

        xTaskCreatePinnedToCore(http_watchdog_task,
                                "http_watchdog",
                                4096,
                                NULL,
                                3,
                                NULL,
                                0);

        if (!s_frame_mutex) {
            ESP_LOGW(TAG, "camera not ready: snapshot/stream endpoints will return errors");
        }
    } else {
        ESP_LOGW(TAG, "Wi-Fi not ready, HTTP server not started");
    }
}
