/**
 * @file lsm9ds1_hal.c
 * @brief High-level HAL for LSM9DS1 (IMU + Magnetometer)
 *
 * Implements the platform-specific I2C read/write functions required by the
 * STMicroelectronics driver and exposes simple init/read functions.
 *
 * Based on: https://github.com/STMicroelectronics/lsm9ds1-pid
 */

#include "lsm9ds1_hal.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "lsm9ds1_reg.h"

#include <stdlib.h>
#include <string.h>

static const char *TAG = "LSM9DS1_HAL";

// Default I2C addresses
#define LSM9DS1_IMU_ADDR 0x6B // Accelerometer/Gyroscope
#define LSM9DS1_MAG_ADDR 0x1E // Magnetometer

// Global I2C port used by this HAL
static i2c_port_t g_i2c_port = I2C_NUM_0;

// ST driver contexts
static stmdev_ctx_t dev_ctx_imu;
static stmdev_ctx_t dev_ctx_mag;

// Device addresses passed as handles to ST driver
static uint8_t imu_addr = LSM9DS1_IMU_ADDR;
static uint8_t mag_addr = LSM9DS1_MAG_ADDR;

/* ---------- Platform I2C access functions ---------- */

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len) {
  uint8_t dev_addr = *(uint8_t *)handle;

  // Allocate buffer: [reg][data...]
  uint8_t *data = (uint8_t *)malloc(len + 1);
  if (data == NULL) {
    return -1;
  }

  data[0] = reg;
  memcpy(&data[1], bufp, len);

  esp_err_t ret = i2c_master_write_to_device(g_i2c_port, dev_addr, data,
                                             len + 1, pdMS_TO_TICKS(100));

  free(data);
  return (ret == ESP_OK) ? 0 : -1;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len) {
  uint8_t dev_addr = *(uint8_t *)handle;

  esp_err_t ret = i2c_master_write_read_device(g_i2c_port, dev_addr, &reg, 1,
                                               bufp, len, pdMS_TO_TICKS(100));

  return (ret == ESP_OK) ? 0 : -1;
}

/* ---------- Public init function ---------- */

esp_err_t lsm9ds1_init(i2c_port_t i2c_num) {
  g_i2c_port = i2c_num;

  // Hook platform functions for IMU
  dev_ctx_imu.write_reg = platform_write;
  dev_ctx_imu.read_reg = platform_read;
  dev_ctx_imu.handle = &imu_addr;

  // Hook platform functions for Magnetometer
  dev_ctx_mag.write_reg = platform_write;
  dev_ctx_mag.read_reg = platform_read;
  dev_ctx_mag.handle = &mag_addr;

  // Read device IDs
  lsm9ds1_id_t whoami;
  if (lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoami) != 0) {
    ESP_LOGE(TAG, "Failed to read LSM9DS1 device IDs");
    return ESP_FAIL;
  }

  if (whoami.imu != LSM9DS1_IMU_ID) {
    ESP_LOGE(TAG, "IMU not found! ID: 0x%02X (expected 0x%02X)", whoami.imu,
             LSM9DS1_IMU_ID);
    return ESP_FAIL;
  }

  if (whoami.mag != LSM9DS1_MAG_ID) {
    ESP_LOGE(TAG, "Mag not found! ID: 0x%02X (expected 0x%02X)", whoami.mag,
             LSM9DS1_MAG_ID);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "LSM9DS1 detected (IMU=0x%02X, MAG=0x%02X)", whoami.imu,
           whoami.mag);

  // --- Reset to default configuration ---
  uint8_t rst;
  lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);
  do {
    lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
  } while (rst);

  // --- Accelerometer: ±4 g, 10 Hz ---
  lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
  lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_14Hz9);

  // --- Gyroscope: ±2000 dps, 10 Hz ---
  lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);

  // --- Magnetometer: ±4 gauss, 10 Hz, continuous mode ---
  lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_4Ga);
  lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_LP_10Hz);

  ESP_LOGI(TAG, "LSM9DS1 configured (XL/GY/MAG @ ~10 Hz)");
  return ESP_OK;
}

/* ---------- Public read functions ---------- */

esp_err_t lsm9ds1_read_accel(float *x, float *y, float *z) {
  uint8_t status;
  lsm9ds1_xl_flag_data_ready_get(&dev_ctx_imu, &status);

  if (status) {
    int16_t data_raw[3];
    lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw);
    *x = lsm9ds1_from_fs4g_to_mg(data_raw[0]);
    *y = lsm9ds1_from_fs4g_to_mg(data_raw[1]);
    *z = lsm9ds1_from_fs4g_to_mg(data_raw[2]);
    return ESP_OK;
  }

  return ESP_ERR_NOT_FOUND; // No data ready
}

esp_err_t lsm9ds1_read_gyro(float *x, float *y, float *z) {
  uint8_t status;
  lsm9ds1_gy_flag_data_ready_get(&dev_ctx_imu, &status);

  if (status) {
    int16_t data_raw[3];
    lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw);
    *x = lsm9ds1_from_fs2000dps_to_mdps(data_raw[0]) / 1000.0f;
    *y = lsm9ds1_from_fs2000dps_to_mdps(data_raw[1]) / 1000.0f;
    *z = lsm9ds1_from_fs2000dps_to_mdps(data_raw[2]) / 1000.0f;
    return ESP_OK;
  }

  return ESP_ERR_NOT_FOUND;
}

esp_err_t lsm9ds1_read_mag(float *x, float *y, float *z) {
  uint8_t status;
  lsm9ds1_mag_flag_data_ready_get(&dev_ctx_mag, &status);

  if (status) {
    int16_t data_raw[3];
    lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw);
    *x = lsm9ds1_from_fs4gauss_to_mG(data_raw[0]) / 1000.0f;
    *y = lsm9ds1_from_fs4gauss_to_mG(data_raw[1]) / 1000.0f;
    *z = lsm9ds1_from_fs4gauss_to_mG(data_raw[2]) / 1000.0f;
    return ESP_OK;
  }

  return ESP_ERR_NOT_FOUND;
}
