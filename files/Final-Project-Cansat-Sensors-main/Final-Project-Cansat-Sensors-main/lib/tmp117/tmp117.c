#include "tmp117.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_TIMEOUT_MS 1000
#define I2C_PORT_NUM I2C_NUM_0

static const char *TAG = "TMP117";


// Initialize TMP117 sensor
void tmp117_init(void) {
    // Configuration Data: Register Address + MSB + LSB
    // 0x01 = Configuration Register
    // 0x0220 = Example settings (Continuous conversion, etc.)
    uint8_t config_data[] = {TMP117_CONFIG_REG, 0x02, 0x20}; 

    esp_err_t err = i2c_master_write_to_device(
        I2C_PORT_NUM, 
        TMP117_ADDR, 
        config_data, 
        sizeof(config_data), 
        pdMS_TO_TICKS(I2C_TIMEOUT_MS)
    );

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "TMP117 initialized successfully.");
    } else {
        ESP_LOGE(TAG, "Failed to initialize TMP117. Error: %s", esp_err_to_name(err));
    }
}

// Read raw temperature data from TMP117
int16_t tmp117_read_raw(void) {
    uint8_t reg = TMP117_TEMP_REG;
    uint8_t data[2];

    esp_err_t err = i2c_master_write_read_device(
        I2C_PORT_NUM,
        TMP117_ADDR,
        &reg,
        1,
        data,
        2,
        pdMS_TO_TICKS(I2C_TIMEOUT_MS)
    );

    if (err != ESP_OK)
    {
        ESP_LOGE (TAG, "Failed to read temperature. Error: %s", esp_err_to_name(err));
        return 0;
    }

    // Combine MSB and LSB
    // TMP117 data is big-endian (MSB first)
    return (int16_t)((data[0] << 8) | data[1]);
}

// Convert raw temperature to Celsius
float tmp117_compensate(int16_t raw_temp) {
    return raw_temp * 0.0078125f; // Convert to Celsius
}