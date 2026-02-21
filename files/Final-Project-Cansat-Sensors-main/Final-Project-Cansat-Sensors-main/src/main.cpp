#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "cJSON.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/i2c.h"
#include "esp_log.h"

// --- RadioLib Includes ---
#include <RadioLib.h>
#include "EspHal.h"

// --- Sensor Includes ---
#include "2d_velocity.h"
#include "dps310.h"
#include "lsm9ds1_hal.h"
#include "neo6m.h"
#include "scd41.h"
#include "tmp117.h"

struct Data 
{
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float pressure;
  int co_2;
  float temp;
  float velocityX, velocityY, velocityZ;
  float altitude;

};


// Mutex handle to protect the data
SemaphoreHandle_t dataMutex = NULL;
Data transmittedData; 


// ===================== PIN DEFINITIONS =====================

// I2C Pins (DPS310, SCD41, LSM9DS1)
#define I2C_PORT       I2C_NUM_0
#define SDA_GPIO       21
#define SCL_GPIO       20

// LoRa Pins (SX1276)
#define LORA_SCK       18
#define LORA_MISO      19
#define LORA_MOSI      23
#define LORA_CS        5
#define LORA_RST       14
#define LORA_DIO0      26  // G0
#define LORA_DIO1      33  // G1

#define LED_PIN GPIO_NUM_2

volatile bool take_picture_flag = false;

static inline void blink_led_ms(int ms) {
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(ms));
    gpio_set_level(LED_PIN, 0);
}
// ===================== LOGGING TAGS =====================

static const char *TAG_MAIN  = "MAIN";
static const char *TAG_BARO  = "BARO";
static const char *TAG_GPS   = "GPS";
static const char *TAG_SCD41 = "SCD41";
static const char *TAG_LORA  = "LORA";
static const char *TAG_IMU   = "IMU";
static const char *TAG_TMP   = "TMP";

// ===================== I2C INIT =====================

static void i2c_init(void) {
    i2c_config_t c = {}; 
    c.mode = I2C_MODE_MASTER;
    c.sda_io_num = SDA_GPIO;
    c.scl_io_num = SCL_GPIO;
    c.sda_pullup_en = GPIO_PULLUP_ENABLE;
    c.scl_pullup_en = GPIO_PULLUP_ENABLE;
    c.master.clk_speed = 400000;
    c.clk_flags = 0;

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &c));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, c.mode, 0, 0, 0));
}

// ===================== LORA TASK =====================

static void lora_task(void *arg) {
    ESP_LOGI(TAG_LORA, "Setting up LoRa Hardware...");

    // 1. Initialize HAL (SPI)
    EspHal* hal = new EspHal(LORA_SCK, LORA_MISO, LORA_MOSI);

    // 2. Initialize Radio Module
    // Note: We use 'new' to allocate on the heap and pass the HAL
    SX1276* radio = new SX1276(new Module(hal, LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1));

    // 3. Begin Radio
    ESP_LOGI(TAG_LORA, "[SX1276] Initializing ...");
    int state = radio->begin();
    
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG_LORA, "radio->begin() failed, code %d", state);
        // If radio fails, we delete the task or loop indefinitely
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_LOGI(TAG_LORA, "radio->begin() success!");

    // Buffer for the JSON string
    char tx_buffer[256];
    // 4. Transmission Loop
    while (1) {
    // 1. Create a local copy of data to minimize mutex holding time
        Data localData;
        
        // Take Mutex
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            localData = transmittedData; // Copy global to local
            xSemaphoreGive(dataMutex);   // Release Mutex
        }

        // 2. Format as JSON (Matching your Receiver's keys!)
        // Keys: ax, ay, az, gx, gy, gz, press, alt, co2, vx, vy
        snprintf(tx_buffer, sizeof(tx_buffer), 
    "{\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
    "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
    "\"press\":%.2f,\"alt\":%.2f,\"co2\":%d,"
    "\"temp\":%.2f,"
    "\"vx\":%.2f,\"vy\":%.2f}",
    localData.accelX, localData.accelY, localData.accelZ,
    localData.gyroX, localData.gyroY, localData.gyroZ,
    localData.pressure, localData.altitude, localData.co_2,
    localData.temp, localData.velocityX, localData.velocityY
);

        ESP_LOGI(TAG_LORA, "Sending: %s", tx_buffer);

        // 3. Transmit
        state = radio->transmit(tx_buffer);

        if (state == RADIOLIB_ERR_NONE) {
            ESP_LOGI(TAG_LORA, "TX success!");
        } else {
            ESP_LOGE(TAG_LORA, "TX failed, code %d", state);
        }

        // Wait for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ===================== BAROMETER TASK =====================

static void baro_task(void *arg) {
    dps310_t dps;
    ESP_LOGI(TAG_BARO, "Initializing DPS310...");

    if (dps310_init(&dps, I2C_PORT, 0x77) != ESP_OK) {
        ESP_LOGE(TAG_BARO, "Failed to initialize DPS310");
        vTaskDelete(NULL);
    }

    if (dps310_config(&dps, DPS310_RATE_16HZ, DPS310_OSR_16, DPS310_RATE_16HZ,
                      DPS310_OSR_16, DPS310_MODE_CONT_PT) != ESP_OK) {
        ESP_LOGE(TAG_BARO, "Failed to configure DPS310");
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG_BARO, "DPS310 configured.");

    while (1) {
        float t_c = 0.0f;
        float p_hpa = 0.0f;

        if (dps310_read(&dps, &t_c, &p_hpa) == ESP_OK) 
        {
            float alt_m = 44330.0f * (1.0f - powf(p_hpa / 1013.25f, 0.1903f));
            ESP_LOGI(TAG_BARO, "T=%.2f C  P=%.2f hPa  Alt≈%.1f m", t_c, p_hpa, alt_m);
                // transmittedData.temp     = t_c;

                if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
                {
                    transmittedData.pressure = p_hpa;
                    transmittedData.altitude = alt_m;
                    // UNLOCK
                    xSemaphoreGive(dataMutex);
                }
        } 
        
        else 
        {
            ESP_LOGW(TAG_BARO, "Failed to read DPS310");
        }
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
}

// ===================== GPS TASK =====================

static void gps_task(void *arg) {
    ESP_LOGI(TAG_GPS, "Starting GPS task...");
    gps_start(); 

    while (1) {
        raw_nmea();
        float v_north = 0.0f;
        float v_east = 0.0f;

        if (gps_get_ground_velocity_ms(&v_north, &v_east)) {
            ESP_LOGI(TAG_GPS, "v_north=%.2f m/s  v_east=%.2f m/s", v_north, v_east);

            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
                {
                    transmittedData.velocityX = v_north;
                    transmittedData.velocityY = v_east;
                    //UNLOCK
                    xSemaphoreGive(dataMutex);
                }
        } else {
            ESP_LOGW(TAG_GPS, "Could not parse ground velocity");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ===================== SCD41 TASK =====================

static void scd41_task(void *arg) {
    ESP_LOGI(TAG_SCD41, "Starting SCD41 task...");
    scd41_config_t scd41_cfg = SCD41_CONFIG_DEFAULT();
    scd41_cfg.i2c_port = I2C_PORT;

    if (scd41_init(&scd41_cfg) != ESP_OK) {
        ESP_LOGE(TAG_SCD41, "Failed to initialize SCD41");
        vTaskDelete(NULL);
    }
    if (scd41_start_measurement() != ESP_OK) {
        ESP_LOGE(TAG_SCD41, "Failed to start measurement");
        vTaskDelete(NULL);
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        bool data_ready = false;
        if (scd41_data_ready(&data_ready) == ESP_OK && data_ready) {
            scd41_data_t data;
            if (scd41_read_measurement(&data) == ESP_OK) {
                ESP_LOGI(TAG_SCD41, "CO2: %u ppm, Temp: %.2f C, Hum: %.2f %%",
                         data.co2_ppm, data.temperature, data.humidity);

                if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
                {
                    transmittedData.co_2 = data.co2_ppm;
                    //UNLOCK
                    xSemaphoreGive(dataMutex);
                }

            } else {
                ESP_LOGW(TAG_SCD41, "Failed to read measurement");
            }
        }
    }
}

// ===================== TMP117 TASK =====================
static void tmp117_task(void *arg)
{
    float raw_temperature = 0.0f;

    ESP_LOGI(TAG_IMU, "Starting TMP117 task...");

    tmp117_init();

    while(1)
    {
        raw_temperature = tmp117_read_raw();
        raw_temperature = tmp117_compensate(raw_temperature);

        ESP_LOGI(TAG_TMP, "Temperature: %.2f ", raw_temperature);

        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
            transmittedData.temp = raw_temperature;
            //UNLOCK
            xSemaphoreGive(dataMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


// ===================== LSM9DS1 TASK =====================

static void lsm9ds1_task(void *arg) {
    ESP_LOGI(TAG_IMU, "Starting LSM9DS1 task...");

    if (lsm9ds1_init(I2C_PORT) != ESP_OK) {
        ESP_LOGE(TAG_IMU, "Failed to initialize LSM9DS1");
        vTaskDelete(NULL);
    }

    // ---- Motion detection settings ----
    const float alpha = 0.95f;             // smoothing for baseline gravity estimate
    const float threshold_g = 0.40f;       // trigger sensitivity (try 0.30–0.60)
    const int64_t cooldown_us = 700000;    // 0.7s cooldown

    float z_base_g = 0.0f;                 // baseline estimate (learns over time)
    bool baseline_inited = false;
    int64_t last_trigger = 0;

    while (1) {
        float ax_mg, ay_mg, az_mg;
        float gx, gy, gz;

        if (lsm9ds1_read_accel(&ax_mg, &ay_mg, &az_mg) == ESP_OK) {

            // Convert mg -> g
            float az_g = az_mg / 1000.0f;

            // Initialize baseline on first good read
            if (!baseline_inited) {
                z_base_g = az_g;
                baseline_inited = true;
            } else {
                // Low-pass baseline = "gravity-ish" component
                z_base_g = alpha * z_base_g + (1.0f - alpha) * az_g;
            }

            // High-pass dynamic acceleration
            float dyn_z = az_g - z_base_g;

            // Trigger with cooldown
            int64_t now = esp_timer_get_time();
            if (fabsf(dyn_z) > threshold_g && (now - last_trigger) > cooldown_us) {
                last_trigger = now;

                blink_led_ms(80);
                take_picture_flag = true;

                ESP_LOGI(TAG_IMU, "MOTION TRIGGER dyn_z=%.2fg -> blink + picture flag", dyn_z);
            }

            // Store accel (mg) for LoRa
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                transmittedData.accelX = ax_mg;
                transmittedData.accelY = ay_mg;
                transmittedData.accelZ = az_mg;
                xSemaphoreGive(dataMutex);
            }
        }

        if (lsm9ds1_read_gyro(&gx, &gy, &gz) == ESP_OK) {
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                transmittedData.gyroX = gx;
                transmittedData.gyroY = gy;
                transmittedData.gyroZ = gz;
                xSemaphoreGive(dataMutex);
            }
        }

        // Faster sampling for motion detection
        vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz
    }
}


// ===================== APP MAIN =====================

extern "C" void app_main(void) {
    // ---------------- LED INIT ----------------
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << (int)LED_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_level(LED_PIN, 0);  // start OFF
    // ------------------------------------------
    esp_log_level_set("*", ESP_LOG_INFO);

    // ---------------------------------------------------------
    // STEP 1: CRITICAL - Create the Mutex BEFORE anything else
    // ---------------------------------------------------------
    dataMutex = xSemaphoreCreateMutex();

    // Check if it failed (e.g., out of memory)
    if (dataMutex == NULL) {
        ESP_LOGE("MAIN", "CRITICAL ERROR: Could not create Mutex!");
        return; // Stop here, do not create tasks
    }

    // ---------------------------------------------------------
    // STEP 2: Initialize I2C (Shared Bus)
    // ---------------------------------------------------------
    i2c_init();

    ESP_LOGI(TAG_MAIN, "Creating tasks...");

    // ---------------------------------------------------------
    // STEP 3: Create Tasks
    // ---------------------------------------------------------
    // GPS Task
    xTaskCreatePinnedToCore(gps_task, "gps_task", 4096, NULL, 5, NULL, 1);
    
    // IMU Task
    xTaskCreatePinnedToCore(lsm9ds1_task, "lsm9ds1_task", 4096, NULL, 8, NULL, 1);
    
    // Barometer Task
    xTaskCreatePinnedToCore(baro_task, "baro_task", 4096, NULL, 4, NULL, 1);
    
    // SCD41 Task
    xTaskCreatePinnedToCore(scd41_task, "scd41_task", 4096, NULL, 4, NULL, 1);
    
    // LoRa Task
    xTaskCreatePinnedToCore(lora_task, "lora_task", 5120, NULL, 10, NULL, 1);

    // TMP117 Task
    xTaskCreatePinnedToCore(tmp117_task, "tpm117_task", 4096, NULL, 8, NULL, 1);
}