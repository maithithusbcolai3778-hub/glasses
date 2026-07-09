/*
 * ATK-MS601M / ATK-IMU901 六轴姿态模块 UART 驱动实现
 */

#include "imu_atk_ms601m.h"

#include <string.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_check.h"

#define TAG "imu_atk_ms601m"

#define IMU_FRAME_START1 0x55
#define IMU_FRAME_START2 0x55
#define IMU_MAX_DATA_LEN 28

#define IMU_UP_ATTITUDE    0x01
#define IMU_UP_QUAT        0x02
#define IMU_UP_GYROACC     0x03

/* 默认量程，仅用于原始数据换算。模块上电后可通过寄存器读取，
 * 这里按常见默认值：陀螺仪 ±2000dps，加速度 ±4g */
#define IMU_DEFAULT_GYRO_FSR_DPS 2000
#define IMU_DEFAULT_ACC_FSR_G    4

typedef enum {
    IMU_STATE_START1,
    IMU_STATE_START2,
    IMU_STATE_ID,
    IMU_STATE_LEN,
    IMU_STATE_DATA,
    IMU_STATE_CHECKSUM,
} imu_parse_state_t;

typedef struct {
    uint8_t start1;
    uint8_t start2;
    uint8_t id;
    uint8_t len;
    uint8_t data[IMU_MAX_DATA_LEN];
    uint8_t checksum;
} imu_frame_t;

static struct {
    bool initialized;
    uart_port_t uart_num;
    TaskHandle_t task_handle;
    SemaphoreHandle_t mutex;

    imu_attitude_t attitude;
    imu_quaternion_t quaternion;
    imu_motion_t motion;
    imu_stats_t stats;

    imu_parse_state_t state;
    imu_frame_t rx_frame;
    uint8_t data_index;
    uint8_t checksum;
} s_imu = {0};

static inline float int16_to_angle(int16_t raw)
{
    return (float)raw / 32768.0f * 180.0f;
}

static inline float int16_to_quat(int16_t raw)
{
    return (float)raw / 32768.0f;
}

static void imu_parse_frame(const imu_frame_t *frame)
{
    if (!frame || frame->len > IMU_MAX_DATA_LEN) {
        return;
    }

    switch (frame->id) {
    case IMU_UP_ATTITUDE: {
        if (frame->len < 6) {
            break;
        }
        int16_t roll  = (int16_t)((frame->data[1] << 8) | frame->data[0]);
        int16_t pitch = (int16_t)((frame->data[3] << 8) | frame->data[2]);
        int16_t yaw   = (int16_t)((frame->data[5] << 8) | frame->data[4]);

        if (xSemaphoreTake(s_imu.mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            s_imu.attitude.roll  = int16_to_angle(roll);
            s_imu.attitude.pitch = int16_to_angle(pitch);
            s_imu.attitude.yaw   = int16_to_angle(yaw);
            s_imu.stats.attitude_frames++;
            xSemaphoreGive(s_imu.mutex);
        }
        break;
    }

    case IMU_UP_QUAT: {
        if (frame->len < 8) {
            break;
        }
        int16_t q0 = (int16_t)((frame->data[1] << 8) | frame->data[0]);
        int16_t q1 = (int16_t)((frame->data[3] << 8) | frame->data[2]);
        int16_t q2 = (int16_t)((frame->data[5] << 8) | frame->data[4]);
        int16_t q3 = (int16_t)((frame->data[7] << 8) | frame->data[6]);

        if (xSemaphoreTake(s_imu.mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            s_imu.quaternion.q0 = int16_to_quat(q0);
            s_imu.quaternion.q1 = int16_to_quat(q1);
            s_imu.quaternion.q2 = int16_to_quat(q2);
            s_imu.quaternion.q3 = int16_to_quat(q3);
            s_imu.stats.quat_frames++;
            xSemaphoreGive(s_imu.mutex);
        }
        break;
    }

    case IMU_UP_GYROACC: {
        if (frame->len < 12) {
            break;
        }
        int16_t acc[3];
        int16_t gyro[3];
        for (int i = 0; i < 3; i++) {
            acc[i]  = (int16_t)((frame->data[i * 2 + 1] << 8) | frame->data[i * 2]);
            gyro[i] = (int16_t)((frame->data[i * 2 + 7] << 8) | frame->data[i * 2 + 6]);
        }

        if (xSemaphoreTake(s_imu.mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            for (int i = 0; i < 3; i++) {
                s_imu.motion.acc_g[i]    = (float)acc[i]  / 32768.0f * IMU_DEFAULT_ACC_FSR_G;
                s_imu.motion.gyro_dps[i] = (float)gyro[i] / 32768.0f * IMU_DEFAULT_GYRO_FSR_DPS;
            }
            s_imu.stats.motion_frames++;
            xSemaphoreGive(s_imu.mutex);
        }
        break;
    }

    default:
        break;
    }

    if (xSemaphoreTake(s_imu.mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_imu.stats.frames_parsed++;
        xSemaphoreGive(s_imu.mutex);
    }
}

static void imu_feed_byte(uint8_t ch)
{
    switch (s_imu.state) {
    case IMU_STATE_START1:
        if (ch == IMU_FRAME_START1) {
            s_imu.rx_frame.start1 = ch;
            s_imu.checksum = ch;
            s_imu.state = IMU_STATE_START2;
        }
        break;

    case IMU_STATE_START2:
        if (ch == IMU_FRAME_START2) {
            s_imu.rx_frame.start2 = ch;
            s_imu.checksum += ch;
            s_imu.state = IMU_STATE_ID;
        } else {
            s_imu.state = IMU_STATE_START1;
        }
        break;

    case IMU_STATE_ID:
        s_imu.rx_frame.id = ch;
        s_imu.checksum += ch;
        s_imu.state = IMU_STATE_LEN;
        break;

    case IMU_STATE_LEN:
        if (ch <= IMU_MAX_DATA_LEN) {
            s_imu.rx_frame.len = ch;
            s_imu.data_index = 0;
            s_imu.checksum += ch;
            s_imu.state = (ch == 0) ? IMU_STATE_CHECKSUM : IMU_STATE_DATA;
        } else {
            s_imu.state = IMU_STATE_START1;
        }
        break;

    case IMU_STATE_DATA:
        s_imu.rx_frame.data[s_imu.data_index++] = ch;
        s_imu.checksum += ch;
        if (s_imu.data_index >= s_imu.rx_frame.len) {
            s_imu.state = IMU_STATE_CHECKSUM;
        }
        break;

    case IMU_STATE_CHECKSUM:
        s_imu.rx_frame.checksum = ch;
        if (s_imu.checksum == ch) {
            imu_parse_frame(&s_imu.rx_frame);
        } else {
            if (xSemaphoreTake(s_imu.mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                s_imu.stats.checksum_errors++;
                xSemaphoreGive(s_imu.mutex);
            }
        }
        s_imu.state = IMU_STATE_START1;
        break;

    default:
        s_imu.state = IMU_STATE_START1;
        break;
    }
}

static void imu_uart_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "imu uart task started on UART%d", (int)s_imu.uart_num);

    uint8_t rx_buf[128];
    while (1) {
        int len = uart_read_bytes(s_imu.uart_num, rx_buf, sizeof(rx_buf),
                                  pdMS_TO_TICKS(50));
        if (len > 0) {
            if (xSemaphoreTake(s_imu.mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                s_imu.stats.bytes_received += (uint32_t)len;
                xSemaphoreGive(s_imu.mutex);
            }
            for (int i = 0; i < len; i++) {
                imu_feed_byte(rx_buf[i]);
            }
        }
    }
}

esp_err_t imu_atk_ms601m_init(void)
{
    if (s_imu.initialized) {
        return ESP_OK;
    }

    if (!CONFIG_IMU_ATK_MS601M_ENABLE) {
        ESP_LOGW(TAG, "IMU disabled in menuconfig");
        return ESP_OK;
    }

    s_imu.mutex = xSemaphoreCreateMutex();
    if (!s_imu.mutex) {
        return ESP_ERR_NO_MEM;
    }

    s_imu.uart_num = (uart_port_t)CONFIG_IMU_ATK_MS601M_UART_NUM;

    uart_config_t uart_config = {
        .baud_rate = CONFIG_IMU_ATK_MS601M_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if SOC_UART_SUPPORT_REF_TICK
        .source_clk = UART_SCLK_REF_TICK,
#else
        .source_clk = UART_SCLK_DEFAULT,
#endif
    };

    int tx_pin = CONFIG_IMU_ATK_MS601M_TX_GPIO;
    int rx_pin = CONFIG_IMU_ATK_MS601M_RX_GPIO;

    ESP_LOGI(TAG, "init UART%d tx=%d rx=%d baud=%d",
             (int)s_imu.uart_num, tx_pin, rx_pin, uart_config.baud_rate);

    ESP_RETURN_ON_ERROR(uart_param_config(s_imu.uart_num, &uart_config),
                        TAG, "uart_param_config failed");

    ESP_RETURN_ON_ERROR(uart_set_pin(s_imu.uart_num, tx_pin, rx_pin,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin failed");

    const int rx_buf_size = CONFIG_IMU_ATK_MS601M_RX_BUF_SIZE;
    const int tx_buf_size = 256;
    ESP_RETURN_ON_ERROR(uart_driver_install(s_imu.uart_num, rx_buf_size,
                                            tx_buf_size, 0, NULL, 0),
                        TAG, "uart_driver_install failed");

    BaseType_t ret = xTaskCreatePinnedToCore(imu_uart_task,
                                             "imu_uart_task",
                                             CONFIG_IMU_ATK_MS601M_TASK_STACK,
                                             NULL,
                                             CONFIG_IMU_ATK_MS601M_TASK_PRIORITY,
                                             &s_imu.task_handle,
                                             0);
    if (ret != pdPASS) {
        uart_driver_delete(s_imu.uart_num);
        vSemaphoreDelete(s_imu.mutex);
        s_imu.mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_imu.initialized = true;
    ESP_LOGI(TAG, "ATK-MS601M init done");
    return ESP_OK;
}

esp_err_t imu_atk_ms601m_get_attitude(imu_attitude_t *out)
{
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "out is null");
    ESP_RETURN_ON_FALSE(s_imu.mutex != NULL, ESP_FAIL, TAG, "not initialized");

    if (xSemaphoreTake(s_imu.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    *out = s_imu.attitude;
    xSemaphoreGive(s_imu.mutex);
    return ESP_OK;
}

esp_err_t imu_atk_ms601m_get_quaternion(imu_quaternion_t *out)
{
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "out is null");
    ESP_RETURN_ON_FALSE(s_imu.mutex != NULL, ESP_FAIL, TAG, "not initialized");

    if (xSemaphoreTake(s_imu.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    *out = s_imu.quaternion;
    xSemaphoreGive(s_imu.mutex);
    return ESP_OK;
}

esp_err_t imu_atk_ms601m_get_motion(imu_motion_t *out)
{
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "out is null");
    ESP_RETURN_ON_FALSE(s_imu.mutex != NULL, ESP_FAIL, TAG, "not initialized");

    if (xSemaphoreTake(s_imu.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    *out = s_imu.motion;
    xSemaphoreGive(s_imu.mutex);
    return ESP_OK;
}

esp_err_t imu_atk_ms601m_get_stats(imu_stats_t *out)
{
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "out is null");
    ESP_RETURN_ON_FALSE(s_imu.mutex != NULL, ESP_FAIL, TAG, "not initialized");

    if (xSemaphoreTake(s_imu.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    *out = s_imu.stats;
    xSemaphoreGive(s_imu.mutex);
    return ESP_OK;
}

bool imu_atk_ms601m_is_initialized(void)
{
    return s_imu.initialized;
}

esp_err_t imu_atk_ms601m_deinit(void)
{
    if (!s_imu.initialized) {
        return ESP_OK;
    }

    if (s_imu.task_handle) {
        vTaskDelete(s_imu.task_handle);
        s_imu.task_handle = NULL;
    }

    uart_driver_delete(s_imu.uart_num);

    if (s_imu.mutex) {
        vSemaphoreDelete(s_imu.mutex);
        s_imu.mutex = NULL;
    }

    s_imu.initialized = false;
    return ESP_OK;
}
