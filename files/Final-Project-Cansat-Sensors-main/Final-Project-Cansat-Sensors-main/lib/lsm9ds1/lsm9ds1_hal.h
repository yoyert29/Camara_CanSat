/**
 * @file lsm9ds1_hal.h
 * @brief High-level Hardware Abstraction Layer (HAL) for LSM9DS1 Sensor
 *
 * This file provides a simplified API for initializing and reading data from
 * the LSM9DS1 9-axis IMU. It wraps the low-level driver provided by
 * STMicroelectronics.
 *
 * @reference Based on: https://github.com/STMicroelectronics/lsm9ds1-pid
 * @datasheet https://www.st.com/resource/en/datasheet/lsm9ds1.pdf
 */

#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the LSM9DS1 sensor (IMU + Magnetometer)
 *
 * @param i2c_num I2C port number (e.g., I2C_NUM_0)
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
esp_err_t lsm9ds1_init(i2c_port_t i2c_num);

/**
 * @brief Read Accelerometer data in mg (milli-g)
 *
 * @param x Pointer to store X axis value
 * @param y Pointer to store Y axis value
 * @param z Pointer to store Z axis value
 * @return ESP_OK on success
 */
esp_err_t lsm9ds1_read_accel(float *x, float *y, float *z);

/**
 * @brief Read Gyroscope data in dps (degrees per second)
 *
 * @param x Pointer to store X axis value
 * @param y Pointer to store Y axis value
 * @param z Pointer to store Z axis value
 * @return ESP_OK on success
 */
esp_err_t lsm9ds1_read_gyro(float *x, float *y, float *z);

/**
 * @brief Read Magnetometer data in gauss
 *
 * @param x Pointer to store X axis value
 * @param y Pointer to store Y axis value
 * @param z Pointer to store Z axis value
 * @return ESP_OK on success
 */
esp_err_t lsm9ds1_read_mag(float *x, float *y, float *z);

#ifdef __cplusplus
}
#endif
