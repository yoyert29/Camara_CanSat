/**
 * @file scd41.h
 * @brief SCD41 CO2, Temperature, and Humidity Sensor Driver
 *
 * This component provides a driver for the Sensirion SCD41 sensor which measures:
 * - CO2 concentration (ppm)
 * - Temperature (°C)
 * - Relative Humidity (%RH)
 */

#pragma once

#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SCD41 I2C address
 */
#define SCD41_I2C_ADDR          0x62

/**
 * @brief SCD41 sensor data structure
 */
typedef struct {
    uint16_t co2_ppm;        ///< CO2 concentration in parts per million
    float temperature;       ///< Temperature in degrees Celsius
    float humidity;          ///< Relative humidity in percent
    bool data_ready;         ///< Flag indicating if data is valid
} scd41_data_t;

/**
 * @brief SCD41 configuration structure
 */
typedef struct {
    i2c_port_t i2c_port;    ///< I2C port number
    uint32_t timeout_ms;     ///< I2C timeout in milliseconds
} scd41_config_t;

/**
 * @brief Default SCD41 configuration
 */
#define SCD41_CONFIG_DEFAULT() { \
    .i2c_port = I2C_NUM_0, \
    .timeout_ms = 1000, \
}

/**
 * @brief Initialize SCD41 sensor
 *
 * @param config Pointer to SCD41 configuration structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t scd41_init(const scd41_config_t *config);

/**
 * @brief Start periodic measurement
 *
 * This command starts the periodic measurement with a measurement interval of 5 seconds.
 * The sensor will update the measurement data every 5 seconds.
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t scd41_start_measurement(void);

/**
 * @brief Stop periodic measurement
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t scd41_stop_measurement(void);

/**
 * @brief Read measurement data
 *
 * @param data Pointer to structure to store measurement data
 * @return ESP_OK on success, error code on failure
 *
 * @note The sensor provides new data every 5 seconds. Calling this function
 *       more frequently will return the same data.
 */
esp_err_t scd41_read_measurement(scd41_data_t *data);

/**
 * @brief Get sensor serial number
 *
 * @param serial_number Buffer to store 6-byte serial number
 * @return ESP_OK on success, error code on failure
 */
esp_err_t scd41_get_serial_number(uint8_t serial_number[6]);

/**
 * @brief Perform soft reset
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t scd41_reset(void);

/**
 * @brief Check if data is ready for reading
 *
 * @param ready Pointer to boolean to store ready status
 * @return ESP_OK on success, error code on failure
 */
esp_err_t scd41_data_ready(bool *ready);

#ifdef __cplusplus
}
#endif