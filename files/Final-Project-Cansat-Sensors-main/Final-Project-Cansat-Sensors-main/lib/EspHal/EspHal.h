#ifndef ESP_HAL_H
#define ESP_HAL_H

#include <RadioLib.h>

// ESP-IDF
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_log.h"

// Arduino-style macros
#define LOW     (0x0)
#define HIGH    (0x1)
#define INPUT   (0x01)
#define OUTPUT  (0x03)
#define RISING  (0x01)
#define FALLING (0x02)
#define NOP()   asm volatile ("nop")

class EspHal : public RadioLibHal {
  public:
    EspHal(int8_t sck, int8_t miso, int8_t mosi)
      : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING),
        spiSCK(sck), spiMISO(miso), spiMOSI(mosi) {}

    void init() override {
      spiBegin();
    }

    void term() override {
      spiEnd();
    }

    void pinMode(uint32_t pin, uint32_t mode) override {
      if(pin == RADIOLIB_NC) return;

      gpio_config_t conf = {};
      conf.pin_bit_mask = (1ULL << pin);
      conf.mode = (gpio_mode_t)mode;
      conf.pull_up_en = GPIO_PULLUP_DISABLE;
      conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
      conf.intr_type = GPIO_INTR_DISABLE;
      gpio_config(&conf);
    }

    void digitalWrite(uint32_t pin, uint32_t value) override {
      if(pin == RADIOLIB_NC) return;
      gpio_set_level((gpio_num_t)pin, value);
    }

    uint32_t digitalRead(uint32_t pin) override {
      if(pin == RADIOLIB_NC) return 0;
      return gpio_get_level((gpio_num_t)pin);
    }

    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override {
      if(interruptNum == RADIOLIB_NC) return;

      // Install ISR service once (ignore "already installed" error)
      esp_err_t e = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
      (void)e;

      gpio_set_intr_type((gpio_num_t)interruptNum, (gpio_int_type_t)(mode & 0x7));

      // RadioLib expects (void)(void) callback, ESP-IDF uses (void)(void*)
      gpio_isr_handler_add((gpio_num_t)interruptNum, (gpio_isr_t)interruptCb, NULL);
    }

    void detachInterrupt(uint32_t interruptNum) override {
      if(interruptNum == RADIOLIB_NC) return;
      gpio_isr_handler_remove((gpio_num_t)interruptNum);
      gpio_set_intr_type((gpio_num_t)interruptNum, GPIO_INTR_DISABLE);
    }

    void delay(unsigned long ms) override {
      vTaskDelay(pdMS_TO_TICKS(ms));
    }

    void delayMicroseconds(unsigned long us) override {
      uint64_t start = (uint64_t)esp_timer_get_time();
      while(((uint64_t)esp_timer_get_time() - start) < us) {
        NOP();
      }
    }

    unsigned long millis() override {
      return (unsigned long)(esp_timer_get_time() / 1000ULL);
    }

    unsigned long micros() override {
      return (unsigned long)(esp_timer_get_time());
    }

    long pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override {
      if(pin == RADIOLIB_NC) return 0;

      this->pinMode(pin, INPUT);

      uint32_t start = this->micros();
      uint32_t now = start;

      // wait for previous pulse to end
      while(this->digitalRead(pin) == state) {
        now = this->micros();
        if((now - start) > timeout) return 0;
      }

      // wait for pulse to start
      while(this->digitalRead(pin) != state) {
        now = this->micros();
        if((now - start) > timeout) return 0;
      }

      uint32_t pulseStart = this->micros();

      // wait for pulse to end
      while(this->digitalRead(pin) == state) {
        now = this->micros();
        if((now - start) > timeout) return 0;
      }

      return (long)(this->micros() - pulseStart);
    }

    // ---- SPI (ESP-IDF portable implementation) ----

    void spiBegin() {
      // Choose a host; SPI2_HOST is typically fine for S3
      host = SPI2_HOST;

      spi_bus_config_t buscfg = {};
      buscfg.mosi_io_num = spiMOSI;
      buscfg.miso_io_num = spiMISO;
      buscfg.sclk_io_num = spiSCK;
      buscfg.quadwp_io_num = -1;
      buscfg.quadhd_io_num = -1;
      buscfg.max_transfer_sz = 0;

      // Initialize bus (ignore "already initialized")
      esp_err_t err = spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO);
      if(err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE("EspHal", "spi_bus_initialize failed: %d", (int)err);
      }

      spi_device_interface_config_t devcfg = {};
      devcfg.clock_speed_hz = 2000000;      // 2 MHz (safe starting point)
      devcfg.mode = 0;                      // SPI mode 0
      devcfg.spics_io_num = -1;             // CS handled by RadioLib via digitalWrite
      devcfg.queue_size = 1;

      err = spi_bus_add_device(host, &devcfg, &dev);
      if(err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE("EspHal", "spi_bus_add_device failed: %d", (int)err);
      }
    }

    void spiBeginTransaction() {
      // Not required for ESP-IDF driver here
    }

    uint8_t spiTransferByte(uint8_t b) {
      uint8_t rx = 0;

      spi_transaction_t t = {};
      t.length = 8;
      t.tx_buffer = &b;
      t.rx_buffer = &rx;

      esp_err_t err = spi_device_transmit(dev, &t);
      if(err != ESP_OK) {
        ESP_LOGE("EspHal", "spi_device_transmit byte failed: %d", (int)err);
      }

      return rx;
    }

    void spiTransfer(uint8_t* out, size_t len, uint8_t* in) {
      if(len == 0) return;

      spi_transaction_t t = {};
      t.length = (uint32_t)(8 * len);
      t.tx_buffer = out;
      t.rx_buffer = in;

      esp_err_t err = spi_device_transmit(dev, &t);
      if(err != ESP_OK) {
        ESP_LOGE("EspHal", "spi_device_transmit failed: %d", (int)err);
      }
    }

    void spiEndTransaction() {
      // Not required
    }

    void spiEnd() {
      if(dev) {
        spi_bus_remove_device(dev);
        dev = NULL;
      }
      // Don’t free the bus globally; other libs may share it
      // spi_bus_free(host);
    }

  private:
    int8_t spiSCK;
    int8_t spiMISO;
    int8_t spiMOSI;

    spi_host_device_t host = SPI2_HOST;
    spi_device_handle_t dev = NULL;
};

#endif