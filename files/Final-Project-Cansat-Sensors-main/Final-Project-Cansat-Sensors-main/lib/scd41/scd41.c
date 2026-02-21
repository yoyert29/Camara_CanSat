/**
 * @file scd41.c
 * @brief SCD41 CO2, Temperature, and Humidity Sensor Driver Implementation
 */

#include "scd41.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "scd41";

// SCD41 command definitions
#define SCD41_START_MEASUREMENT  0x21B1  // Start periodic measurement
#define SCD41_STOP_MEASUREMENT   0x3F86  // Stop measurement
#define SCD41_READ_MEASUREMENT   0xEC05  // Read measurement
#define SCD41_GET_SERIAL_NUMBER  0x3682  // Get serial number
#define SCD41_RESET              0x3646  // Soft reset
#define SCD41_DATA_READY         0xE4B8  // Get data ready status

// Global configuration
static scd41_config_t g_config;
static bool g_initialized = false;

/**
 * @brief Calculate CRC8 for SCD41 communication
 */
static uint8_t scd41_crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

/**
 * @brief Write a command to SCD41 sensor
 */
static esp_err_t scd41_write_command(uint16_t command)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t cmd_buffer[2];
    cmd_buffer[0] = (command >> 8) & 0xFF;  // High byte
    cmd_buffer[1] = command & 0xFF;         // Low byte

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SCD41_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, cmd_buffer, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(g_config.i2c_port, cmd, pdMS_TO_TICKS(g_config.timeout_ms));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Read data from SCD41 sensor
 */
static esp_err_t scd41_read_data(uint8_t *data, size_t length)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SCD41_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, length, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(g_config.i2c_port, cmd, pdMS_TO_TICKS(g_config.timeout_ms));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t scd41_init(const scd41_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    // Store configuration
    g_config = *config;
    g_initialized = true;

    ESP_LOGI(TAG, "Initializing SCD41 sensor on I2C port %d...", g_config.i2c_port);

    // Stop any ongoing measurement first
    esp_err_t ret = scd41_write_command(SCD41_STOP_MEASUREMENT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop SCD41 measurement: %s", esp_err_to_name(ret));
        g_initialized = false;
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(500));  // Wait for stop command

    ESP_LOGI(TAG, "SCD41 sensor initialized successfully");
    return ESP_OK;
}

esp_err_t scd41_start_measurement(void)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = scd41_write_command(SCD41_START_MEASUREMENT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start SCD41 measurement: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SCD41 measurement started");
    return ESP_OK;
}

esp_err_t scd41_stop_measurement(void)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = scd41_write_command(SCD41_STOP_MEASUREMENT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop SCD41 measurement: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(500));  // Wait for stop command
    ESP_LOGI(TAG, "SCD41 measurement stopped");
    return ESP_OK;
}

esp_err_t scd41_read_measurement(scd41_data_t *data)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }

    // Send read measurement command
    esp_err_t ret = scd41_write_command(SCD41_READ_MEASUREMENT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send read command: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(1));  // Wait for data to be ready

    // Read 9 bytes: 3 measurements * (2 data bytes + 1 CRC byte)
    uint8_t buffer[9];
    ret = scd41_read_data(buffer, 9);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read measurement data: %s", esp_err_to_name(ret));
        return ret;
    }

    // Verify CRC for each measurement
    for (int i = 0; i < 3; i++) {
        uint8_t expected_crc = scd41_crc8(&buffer[i * 3], 2);
        if (buffer[i * 3 + 2] != expected_crc) {
            ESP_LOGE(TAG, "CRC mismatch for measurement %d: expected 0x%02X, got 0x%02X",
                     i, expected_crc, buffer[i * 3 + 2]);
            return ESP_ERR_INVALID_CRC;
        }
    }

    // Parse CO2 concentration (ppm)
    data->co2_ppm = (buffer[0] << 8) | buffer[1];

    // Parse temperature (°C)
    uint16_t temp_raw = (buffer[3] << 8) | buffer[4];
    data->temperature = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);

    // Parse humidity (%RH)
    uint16_t hum_raw = (buffer[6] << 8) | buffer[7];
    data->humidity = 100.0f * ((float)hum_raw / 65535.0f);

    data->data_ready = true;

    ESP_LOGD(TAG, "SCD41 - CO2: %d ppm, Temp: %.1f°C, Humidity: %.1f%%",
             data->co2_ppm, data->temperature, data->humidity);

    return ESP_OK;
}

esp_err_t scd41_get_serial_number(uint8_t serial_number[6])
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!serial_number) {
        return ESP_ERR_INVALID_ARG;
    }

    // Send get serial number command
    esp_err_t ret = scd41_write_command(SCD41_GET_SERIAL_NUMBER);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to request SCD41 serial number: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    uint8_t serial_data[9];  // 3 words * 3 bytes each (2 data + 1 CRC)
    ret = scd41_read_data(serial_data, 9);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read serial number: %s", esp_err_to_name(ret));
        return ret;
    }

    // Verify CRC for each word and extract serial number
    for (int i = 0; i < 3; i++) {
        uint8_t expected_crc = scd41_crc8(&serial_data[i * 3], 2);
        if (serial_data[i * 3 + 2] != expected_crc) {
            ESP_LOGE(TAG, "CRC mismatch for serial word %d", i);
            return ESP_ERR_INVALID_CRC;
        }
        // Copy the 2 data bytes to serial number array
        serial_number[i * 2] = serial_data[i * 3];
        serial_number[i * 2 + 1] = serial_data[i * 3 + 1];
    }

    ESP_LOGI(TAG, "SCD41 serial number: %02X%02X%02X%02X%02X%02X",
             serial_number[0], serial_number[1], serial_number[2],
             serial_number[3], serial_number[4], serial_number[5]);

    return ESP_OK;
}

esp_err_t scd41_reset(void)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = scd41_write_command(SCD41_RESET);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset SCD41: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for reset to complete
    ESP_LOGI(TAG, "SCD41 reset completed");
    return ESP_OK;
}

esp_err_t scd41_data_ready(bool *ready)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!ready) {
        return ESP_ERR_INVALID_ARG;
    }

    // Send data ready command
    esp_err_t ret = scd41_write_command(SCD41_DATA_READY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to check data ready status: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    // Read 3 bytes: 2 data bytes + 1 CRC
    uint8_t buffer[3];
    ret = scd41_read_data(buffer, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data ready status: %s", esp_err_to_name(ret));
        return ret;
    }

    // Verify CRC
    uint8_t expected_crc = scd41_crc8(buffer, 2);
    if (buffer[2] != expected_crc) {
        ESP_LOGE(TAG, "CRC mismatch for data ready status");
        return ESP_ERR_INVALID_CRC;
    }

    // Check if data is ready (bit 11 of the 16-bit word)
    uint16_t status = (buffer[0] << 8) | buffer[1];
    *ready = (status & 0x07FF) > 0;  // Lower 11 bits indicate data ready

    return ESP_OK;
}