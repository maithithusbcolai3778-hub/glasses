/*
 * ATK-MS601M / ATK-IMU901 六轴姿态模块 UART 驱动
 *
 * 协议：0x55 0x55 ID LEN DATA[N] SUM
 * 校验和 = 0x55 + 0x55 + ID + LEN + DATA[0..N-1]
 * 姿态角 ID=0x01，LEN=6：RollL RollH PitchL PitchH YawL YawH
 * 角度 = int16_t((H<<8)|L) / 32768.0 * 180.0
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 姿态角数据结构
 */
typedef struct {
    float roll;   /*!< 横滚角，单位：度 */
    float pitch;  /*!< 俯仰角，单位：度 */
    float yaw;    /*!< 航向角，单位：度 */
} imu_attitude_t;

/**
 * @brief 四元数数据结构
 */
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} imu_quaternion_t;

/**
 * @brief 原始陀螺仪/加速度计数据（已按量程换算）
 */
typedef struct {
    float acc_g[3];   /*!< 加速度，单位：g */
    float gyro_dps[3];/*!< 角速度，单位：°/s */
} imu_motion_t;

/**
 * @brief 模块接收统计
 */
typedef struct {
    uint32_t attitude_frames;  /*!< 姿态角帧数 */
    uint32_t quat_frames;      /*!< 四元数帧数 */
    uint32_t motion_frames;    /*!< 原始数据帧数 */
    uint32_t checksum_errors;  /*!< 校验错误数 */
    uint32_t bytes_received;   /*!< 接收字节数 */
    uint32_t frames_parsed;    /*!< 成功解析帧数 */
} imu_stats_t;

/**
 * @brief 初始化 ATK-MS601M UART 并启动解析任务
 *
 * @return ESP_OK on success
 */
esp_err_t imu_atk_ms601m_init(void);

/**
 * @brief 获取最新姿态角（线程安全）
 *
 * @param out 输出结构体
 * @return ESP_OK on success，ESP_ERR_INVALID_ARG 或 ESP_FAIL
 */
esp_err_t imu_atk_ms601m_get_attitude(imu_attitude_t *out);

/**
 * @brief 获取最新四元数（线程安全）
 *
 * @param out 输出结构体
 * @return ESP_OK on success
 */
esp_err_t imu_atk_ms601m_get_quaternion(imu_quaternion_t *out);

/**
 * @brief 获取最新原始运动数据（线程安全）
 *
 * @param out 输出结构体
 * @return ESP_OK on success
 */
esp_err_t imu_atk_ms601m_get_motion(imu_motion_t *out);

/**
 * @brief 获取接收统计（线程安全）
 *
 * @param out 输出结构体
 * @return ESP_OK on success
 */
esp_err_t imu_atk_ms601m_get_stats(imu_stats_t *out);

/**
 * @brief 返回驱动是否已初始化
 */
bool imu_atk_ms601m_is_initialized(void);

/**
 * @brief 反初始化，释放 UART 与任务
 */
esp_err_t imu_atk_ms601m_deinit(void);

#ifdef __cplusplus
}
#endif
