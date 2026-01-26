#include "ptc06.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_check.h"
#include "driver/uart.h"

static const char *TAG = "ptc06";

/* ==== Protocol Definitions (VC0706 / PTC06) ==== */
[cite_start]/* RESET: 56 00 26 00 -> 76 00 26 00 [cite: 53] */
static const uint8_t CMD_RESET[]   = {0x56, 0x00, 0x26, 0x00};

[cite_start]/* CAPTURE: 56 00 36 01 00 -> 76 00 36 00 00 [cite: 61] */
static const uint8_t CMD_CAPTURE[] = {0x56, 0x00, 0x36, 0x01, 0x00};

[cite_start]/* STOP (Resume): 56 00 36 01 03 -> 76 00 36 00 00 [cite: 94] */
static const uint8_t CMD_RESUME[]  = {0x56, 0x00, 0x36, 0x01, 0x03};

[cite_start]/* GET LENGTH: 56 00 34 01 00 -> 76 00 34 00 04 00 00 XX YY [cite: 69] */
static const uint8_t CMD_LENGTH[]  = {0x56, 0x00, 0x34, 0x01, 0x00};

struct ptc06_t {
    ptc06_config_t cfg;
};

/* ==== Helper Functions ==== */

static esp_err_t uart_read_exact(uart_port_t uart, uint8_t *out, size_t len, int timeout_ms) {
    TickType_t ticks_to_wait = pdMS_TO_TICKS(timeout_ms);
    
    int res = uart_read_bytes(uart, out, len, ticks_to_wait);
    
    if (res < 0) return ESP_FAIL;
    if ((size_t)res != len) {
        ESP_LOGW(TAG, "UART Read timeout. Expected %d, got %d", (int)len, res);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static esp_err_t uart_write_all(uart_port_t uart, const uint8_t *data, size_t len) {
    uart_write_bytes(uart, (const char *)data, len);
    return uart_wait_tx_done(uart, pdMS_TO_TICKS(1000));
}

static esp_err_t expect_prefix(uart_port_t uart, const uint8_t *prefix, size_t plen, int timeout_ms) {
    uint8_t *buf = malloc(plen);
    if (!buf) return ESP_ERR_NO_MEM;

    esp_err_t err = uart_read_exact(uart, buf, plen, timeout_ms);
    if (err == ESP_OK) {
        if (memcmp(buf, prefix, plen) != 0) {
            ESP_LOGW(TAG, "Bad Prefix. Expected 0x%02X, Got 0x%02X", prefix[0], buf[0]);
            err = ESP_FAIL;
        }
    }
    free(buf);
    return err;
}

static esp_err_t ptc06_cmd_simple_ack(ptc06_t *cam, const uint8_t *cmd, size_t cmd_len,
                                     const uint8_t *ack_prefix, size_t ack_len) {
    uart_flush_input(cam->cfg.uart_num);
    ESP_RETURN_ON_ERROR(uart_write_all(cam->cfg.uart_num, cmd, cmd_len), TAG, "write cmd");
    return expect_prefix(cam->cfg.uart_num, ack_prefix, ack_len, cam->cfg.cmd_timeout_ms);
}

static void build_read_cmd(uint32_t addr, uint16_t len, uint8_t *out) {
    [cite_start]/* Protocol: 56 00 32 0C 00 0D 00 00 XX XX 00 00 YY YY 00 0A [cite: 81] */
    out[0]=0x56; out[1]=0x00; out[2]=0x32; out[3]=0x0C;
    out[4]=0x00; out[5]=0x0D; out[6]=0x00; out[7]=0x00;
    out[8]=(addr >> 8) & 0xFF; out[9]=addr & 0xFF;  // XX XX (Start Addr)
    out[10]=0x00; out[11]=0x00;                     // 00 00 (Padding)
    out[12]=(len >> 8) & 0xFF; out[13]=len & 0xFF;  // YY YY (Length)
    out[14]=0x00; out[15]=0x0A;                     // 00 0A (Delay)
}

/* ==== Public API ==== */

esp_err_t ptc06_init(ptc06_t **out_cam, const ptc06_config_t *cfg) {
    if (!out_cam || !cfg) return ESP_ERR_INVALID_ARG;

    ptc06_t *cam = calloc(1, sizeof(ptc06_t));
    if (!cam) return ESP_ERR_NO_MEM;
    cam->cfg = *cfg;

    uart_config_t ucfg = {
        .baud_rate = cfg->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(cfg->uart_num, cfg->uart_rx_buf_size, cfg->uart_tx_buf_size, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(cfg->uart_num, &ucfg));
    ESP_ERROR_CHECK(uart_set_pin(cfg->uart_num, cfg->tx_pin, cfg->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    [cite_start]/* DATASHEET REQUIREMENT: Power on -> delay 2.5s -> reset command [cite: 126] */
    ESP_LOGI(TAG, "Waiting 2.5s for camera boot...");
    vTaskDelay(pdMS_TO_TICKS(2500)); 

    if (ptc06_reset(cam) != ESP_OK) {
        ESP_LOGW(TAG, "Camera did not ACK reset. It might already be awake or baud rate mismatch.");
    } else {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    *out_cam = cam;
    return ESP_OK;
}

esp_err_t ptc06_deinit(ptc06_t *cam) {
    if (!cam) return ESP_ERR_INVALID_ARG;
    uart_driver_delete(cam->cfg.uart_num);
    free(cam);
    return ESP_OK;
}

esp_err_t ptc06_reset(ptc06_t *cam) {
    const uint8_t ACK[] = {0x76, 0x00, 0x26, 0x00}; [cite_start]// [cite: 58]
    return ptc06_cmd_simple_ack(cam, CMD_RESET, sizeof(CMD_RESET), ACK, sizeof(ACK));
}

esp_err_t ptc06_set_resolution(ptc06_t *cam, ptc06_resolution_t res) {
    [cite_start]/* Command: 56 00 31 05 04 01 00 19 XX [cite: 107] */
    uint8_t cmd[] = {0x56, 0x00, 0x31, 0x05, 0x04, 0x01, 0x00, 0x19, (uint8_t)res};
    const uint8_t ACK[] = {0x76, 0x00, 0x31, 0x00, 0x00}; [cite_start]// [cite: 109]
    return ptc06_cmd_simple_ack(cam, cmd, sizeof(cmd), ACK, sizeof(ACK));
}

esp_err_t ptc06_set_compressibility(ptc06_t *cam, uint8_t value) {
    [cite_start]/* Command: 56 00 31 05 01 01 12 04 XX [cite: 98] */
    uint8_t cmd[] = {0x56, 0x00, 0x31, 0x05, 0x01, 0x01, 0x12, 0x04, value};
    const uint8_t ACK[] = {0x76, 0x00, 0x31, 0x00, 0x00}; [cite_start]// [cite: 99]
    return ptc06_cmd_simple_ack(cam, cmd, sizeof(cmd), ACK, sizeof(ACK));
}

esp_err_t ptc06_capture_jpeg_stream(ptc06_t *cam, size_t chunk_size, ptc06_stream_cb_t cb, void *ctx, uint32_t *out_len) {
    if (!cam || !cb) return ESP_ERR_INVALID_ARG;
    
    /* 1. Freeze Frame */
    const uint8_t ACK_CAP[] = {0x76, 0x00, 0x36, 0x00, 0x00}; [cite_start]// [cite: 66]
    ESP_RETURN_ON_ERROR(ptc06_cmd_simple_ack(cam, CMD_CAPTURE, sizeof(CMD_CAPTURE), ACK_CAP, sizeof(ACK_CAP)), TAG, "Capture failed");

    /* 2. Get Length */
    uint8_t len_resp[9] = {0};
    uart_write_all(cam->cfg.uart_num, CMD_LENGTH, sizeof(CMD_LENGTH));
    
    [cite_start]// Expect: 76 00 34 00 04 00 00 XX YY [cite: 70]
    if (uart_read_exact(cam->cfg.uart_num, len_resp, 9, cam->cfg.cmd_timeout_ms) != ESP_OK) {
        ESP_LOGE(TAG, "Get length timeout");
        return ESP_FAIL;
    }
    
    uint32_t jpg_len = ((uint32_t)len_resp[7] << 8) | len_resp[8]; [cite_start]// [cite: 76]
    if (out_len) *out_len = jpg_len;

    /* 3. Read chunks */
    [cite_start]/* "Start address must be times of 8" [cite: 86] */
    chunk_size = (chunk_size < 32) ? 32 : chunk_size;
    chunk_size &= ~7; // Enforces alignment

    uint8_t *rx_buf = heap_caps_malloc(chunk_size, MALLOC_CAP_8BIT);
    if (!rx_buf) return ESP_ERR_NO_MEM;

    const uint8_t HDR_RESP[] = {0x76, 0x00, 0x32, 0x00, 0x00}; [cite_start]// [cite: 82]
    uint32_t addr = 0;
    esp_err_t err = ESP_OK;

    while (addr < jpg_len) {
        uint16_t to_read = (jpg_len - addr);
        if (to_read > chunk_size) to_read = chunk_size;

        uint8_t cmd[16];
        build_read_cmd(addr, to_read, cmd);

        uart_flush_input(cam->cfg.uart_num); 
        uart_write_all(cam->cfg.uart_num, cmd, sizeof(cmd));

        // Response Header: 76 00 32 00 00
        if (expect_prefix(cam->cfg.uart_num, HDR_RESP, 5, cam->cfg.cmd_timeout_ms) != ESP_OK) {
            ESP_LOGE(TAG, "Chunk header error");
            err = ESP_FAIL;
            break;
        }

        // Data
        if (uart_read_exact(cam->cfg.uart_num, rx_buf, to_read, cam->cfg.cmd_timeout_ms) != ESP_OK) {
            err = ESP_FAIL;
            break;
        }

        [cite_start]// Response Tail: 76 00 32 00 00 [cite: 85]
        uint8_t tail[5];
        uart_read_bytes(cam->cfg.uart_num, tail, 5, pdMS_TO_TICKS(20));

        if (cb(rx_buf, to_read, ctx) != ESP_OK) {
            err = ESP_FAIL;
            break;
        }

        addr += to_read;
    }

    free(rx_buf);

    /* 4. Resume Video */
    const uint8_t ACK_RESUME[] = {0x76, 0x00, 0x36, 0x00, 0x00}; [cite_start]// [cite: 94]
    ptc06_cmd_simple_ack(cam, CMD_RESUME, sizeof(CMD_RESUME), ACK_RESUME, sizeof(ACK_RESUME));

    return err;
}

/* Helper context for buffer writer */
typedef struct {
    uint8_t *buffer;
    size_t cursor;
} buffer_ctx_t;

static esp_err_t buffer_writer_cb(const uint8_t *data, size_t len, void *ctx) {
    buffer_ctx_t *b = (buffer_ctx_t *)ctx;
    memcpy(b->buffer + b->cursor, data, len);
    b->cursor += len;
    return ESP_OK;
}

esp_err_t ptc06_capture_jpeg_to_buffer(ptc06_t *cam, uint8_t **out_buf, size_t *out_len) {
    if (!cam || !out_buf || !out_len) return ESP_ERR_INVALID_ARG;

    /* 1. Freeze Frame */
    const uint8_t ACK_CAP[] = {0x76, 0x00, 0x36, 0x00, 0x00}; [cite_start]// [cite: 66]
    ESP_RETURN_ON_ERROR(ptc06_cmd_simple_ack(cam, CMD_CAPTURE, sizeof(CMD_CAPTURE), ACK_CAP, sizeof(ACK_CAP)), TAG, "Capture failed");

    /* 2. Get Length */
    uint8_t len_resp[9] = {0};
    uart_flush_input(cam->cfg.uart_num);
    uart_write_all(cam->cfg.uart_num, CMD_LENGTH, sizeof(CMD_LENGTH));
    
    if (uart_read_exact(cam->cfg.uart_num, len_resp, 9, cam->cfg.cmd_timeout_ms) != ESP_OK) {
        return ESP_FAIL;
    }

    uint32_t total_len = ((uint32_t)len_resp[7] << 8) | len_resp[8]; [cite_start]// [cite: 76]
    if (total_len == 0) return ESP_ERR_INVALID_SIZE;

    /* 3. Allocate Exact Memory */
    /* Try SPIRAM first (PSRAM), fall back to internal memory */
    uint8_t *buf = heap_caps_malloc(total_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        buf = heap_caps_malloc(total_len, MALLOC_CAP_8BIT);
        if (!buf) return ESP_ERR_NO_MEM;
    }

    /* 4. Stream Data into Buffer */
    /* We duplicate the loop logic here because 'ptc06_capture_jpeg_stream' attempts to Freeze/Length again */
    
    // Create a temporary buffer for reading chunks (512 bytes is a safe standard)
    size_t chunk_sz = 512;
    uint8_t *chunk_buf = heap_caps_malloc(chunk_sz, MALLOC_CAP_8BIT);
    if (!chunk_buf) {
        free(buf);
        return ESP_ERR_NO_MEM;
    }

    uint32_t addr = 0;
    const uint8_t HDR_RESP[] = {0x76, 0x00, 0x32, 0x00, 0x00}; [cite_start]// [cite: 82]
    esp_err_t err = ESP_OK;

    while (addr < total_len) {
        uint16_t to_read = (total_len - addr > chunk_sz) ? (uint16_t)chunk_sz : (uint16_t)(total_len - addr);
        uint8_t cmd[16];
        build_read_cmd(addr, to_read, cmd);

        uart_flush_input(cam->cfg.uart_num);
        uart_write_all(cam->cfg.uart_num, cmd, sizeof(cmd));

        // Header
        if (expect_prefix(cam->cfg.uart_num, HDR_RESP, 5, cam->cfg.cmd_timeout_ms) != ESP_OK) {
            err = ESP_FAIL; break;
        }
        // Data
        if (uart_read_exact(cam->cfg.uart_num, buf + addr, to_read, cam->cfg.cmd_timeout_ms) != ESP_OK) {
            err = ESP_FAIL; break;
        }
        
        [cite_start]// Tail [cite: 85]
        uint8_t tail[5];
        uart_read_bytes(cam->cfg.uart_num, tail, 5, pdMS_TO_TICKS(20));

        addr += to_read;
    }

    free(chunk_buf);

    /* 5. Resume Video */
    const uint8_t ACK_RESUME[] = {0x76, 0x00, 0x36, 0x00, 0x00}; [cite_start]// [cite: 94]
    ptc06_cmd_simple_ack(cam, CMD_RESUME, sizeof(CMD_RESUME), ACK_RESUME, sizeof(ACK_RESUME));

    if (err == ESP_OK) {
        *out_buf = buf;
        *out_len = total_len;
    } else {
        free(buf);
        *out_buf = NULL;
        *out_len = 0;
    }

    return err;
}
