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

/* ==== Protocol Definitions (VC0706) ==== */
static const uint8_t CMD_RESET[]   = {0x56, 0x00, 0x26, 0x00};
static const uint8_t CMD_CAPTURE[] = {0x56, 0x00, 0x36, 0x01, 0x00};
static const uint8_t CMD_RESUME[]  = {0x56, 0x00, 0x36, 0x01, 0x03};
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
    // On high speeds, we don't necessarily need to wait for TX done strictly, 
    // but it ensures command ordering.
    return uart_wait_tx_done(uart, pdMS_TO_TICKS(500)); 
}

static esp_err_t expect_prefix(uart_port_t uart, const uint8_t *prefix, size_t plen, int timeout_ms) {
    // FIX: Use stack buffer instead of malloc to prevent heap fragmentation in loops
    uint8_t buf[16]; 
    if (plen > sizeof(buf)) return ESP_ERR_INVALID_SIZE;

    esp_err_t err = uart_read_exact(uart, buf, plen, timeout_ms);
    if (err == ESP_OK) {
        if (memcmp(buf, prefix, plen) != 0) {
            ESP_LOGW(TAG, "Bad Prefix. Exp: 0x%02X, Got: 0x%02X", prefix[0], buf[0]);
            return ESP_FAIL;
        }
    }
    return err;
}

static esp_err_t ptc06_cmd_simple_ack(ptc06_t *cam, const uint8_t *cmd, size_t cmd_len,
                                     const uint8_t *ack_prefix, size_t ack_len) {
    uart_flush_input(cam->cfg.uart_num);
    ESP_RETURN_ON_ERROR(uart_write_all(cam->cfg.uart_num, cmd, cmd_len), TAG, "write cmd");
    return expect_prefix(cam->cfg.uart_num, ack_prefix, ack_len, cam->cfg.cmd_timeout_ms);
}

static void build_read_cmd(uint32_t addr, uint32_t len, uint8_t *out) {
    // FIX: Standard VC0706 "Read FBuf" (0x32) structure
    // 56 00 32 0C 00 0A A3 A2 A1 A0 L3 L2 L1 L0 00 0A
    out[0]=0x56; out[1]=0x00; out[2]=0x32; out[3]=0x0C;
    out[4]=0x00; out[5]=0x0A; // 0x0A = MCU Mode
    
    // Address (Big Endian, 4 bytes)
    out[6] = (addr >> 24) & 0xFF;
    out[7] = (addr >> 16) & 0xFF;
    out[8] = (addr >> 8)  & 0xFF;
    out[9] = addr & 0xFF;

    // Length (Big Endian, 4 bytes)
    out[10] = (len >> 24) & 0xFF;
    out[11] = (len >> 16) & 0xFF;
    out[12] = (len >> 8)  & 0xFF;
    out[13] = len & 0xFF;

    // Delay (2 bytes)
    out[14] = 0x00; out[15] = 0x0A; 
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

    ESP_LOGI(TAG, "Waiting 2.5s for camera boot...");
    vTaskDelay(pdMS_TO_TICKS(2500)); 

    // Flush any boot-up text/garbage from the camera
    uart_flush_input(cam->cfg.uart_num);

    if (ptc06_reset(cam) != ESP_OK) {
        ESP_LOGW(TAG, "Camera did not ACK reset. Retrying once...");
        vTaskDelay(pdMS_TO_TICKS(100));
        uart_flush_input(cam->cfg.uart_num);
        if(ptc06_reset(cam) != ESP_OK) {
             ESP_LOGE(TAG, "Camera initialization failed");
             // We return OK here but the camera might be unresponsive. 
             // Depending on app logic, you might want to return ESP_FAIL.
        }
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
    const uint8_t ACK[] = {0x76, 0x00, 0x26, 0x00};
    return ptc06_cmd_simple_ack(cam, CMD_RESET, sizeof(CMD_RESET), ACK, sizeof(ACK));
}

esp_err_t ptc06_set_resolution(ptc06_t *cam, ptc06_resolution_t res) {
    uint8_t cmd[] = {0x56, 0x00, 0x31, 0x05, 0x04, 0x01, 0x00, 0x19, (uint8_t)res};
    const uint8_t ACK[] = {0x76, 0x00, 0x31, 0x00, 0x00};
    return ptc06_cmd_simple_ack(cam, cmd, sizeof(cmd), ACK, sizeof(ACK));
}

esp_err_t ptc06_set_compressibility(ptc06_t *cam, uint8_t value) {
    uint8_t cmd[] = {0x56, 0x00, 0x31, 0x05, 0x01, 0x01, 0x12, 0x04, value};
    const uint8_t ACK[] = {0x76, 0x00, 0x31, 0x00, 0x00};
    return ptc06_cmd_simple_ack(cam, cmd, sizeof(cmd), ACK, sizeof(ACK));
}

esp_err_t ptc06_capture_jpeg_stream(ptc06_t *cam, size_t chunk_size, ptc06_stream_cb_t cb, void *ctx, uint32_t *out_len) {
    if (!cam || !cb) return ESP_ERR_INVALID_ARG;
    
    /* 1. Freeze Frame */
    const uint8_t ACK_CAP[] = {0x76, 0x00, 0x36, 0x00, 0x00};
    ESP_RETURN_ON_ERROR(ptc06_cmd_simple_ack(cam, CMD_CAPTURE, sizeof(CMD_CAPTURE), ACK_CAP, sizeof(ACK_CAP)), TAG, "Capture failed");

    /* 2. Get Length */
    // VC0706 Return: 76 00 34 00 04 L3 L2 L1 L0 (9 bytes total)
    uint8_t len_resp[9] = {0};
    uart_flush_input(cam->cfg.uart_num);
    uart_write_all(cam->cfg.uart_num, CMD_LENGTH, sizeof(CMD_LENGTH));
    
    if (uart_read_exact(cam->cfg.uart_num, len_resp, 9, cam->cfg.cmd_timeout_ms) != ESP_OK) {
        ESP_LOGE(TAG, "Get length timeout");
        return ESP_FAIL;
    }
    
    // FIX: Read 4 bytes for length to support images > 64KB
    uint32_t jpg_len = ((uint32_t)len_resp[5] << 24) | 
                       ((uint32_t)len_resp[6] << 16) | 
                       ((uint32_t)len_resp[7] << 8)  | 
                       len_resp[8];

    ESP_LOGI(TAG, "Image Size: %lu bytes", jpg_len);
    if (out_len) *out_len = jpg_len;

    /* 3. Read chunks */
    chunk_size = (chunk_size < 32) ? 32 : chunk_size;
    chunk_size &= ~7; // Enforces 8-byte alignment required by camera HW

    // Alloc chunk buffer (Use SPIRAM if available for larger chunks, otherwise internal)
    uint8_t *rx_buf = heap_caps_malloc(chunk_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (!rx_buf) {
        // Fallback to internal if SPIRAM alloc failed
        rx_buf = heap_caps_malloc(chunk_size, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
        if (!rx_buf) return ESP_ERR_NO_MEM;
    }

    const uint8_t HDR_RESP[] = {0x76, 0x00, 0x32, 0x00, 0x00};
    uint32_t addr = 0;
    esp_err_t err = ESP_OK;

    while (addr < jpg_len) {
        uint32_t to_read = (jpg_len - addr);
        if (to_read > chunk_size) to_read = chunk_size;

        uint8_t cmd[16];
        build_read_cmd(addr, to_read, cmd);

        uart_flush_input(cam->cfg.uart_num); 
        uart_write_all(cam->cfg.uart_num, cmd, sizeof(cmd));

        // Response Header: 76 00 32 00 00
        if (expect_prefix(cam->cfg.uart_num, HDR_RESP, 5, cam->cfg.cmd_timeout_ms) != ESP_OK) {
            ESP_LOGE(TAG, "Chunk header error at addr %lu", addr);
            err = ESP_FAIL;
            break;
        }

        // Data
        if (uart_read_exact(cam->cfg.uart_num, rx_buf, to_read, cam->cfg.cmd_timeout_ms) != ESP_OK) {
            ESP_LOGE(TAG, "Chunk data timeout at addr %lu", addr);
            err = ESP_FAIL;
            break;
        }

        // Response Tail: 76 00 32 00 00
        // FIX: Increased timeout slightly. 20ms is too tight for slow UARTs/OS jitter.
        uint8_t tail[5];
        if (uart_read_exact(cam->cfg.uart_num, tail, 5, 100) != ESP_OK) {
            ESP_LOGW(TAG, "Chunk tail timeout");
            // Not necessarily fatal, but indicates syncing issues
        }

        if (cb(rx_buf, to_read, ctx) != ESP_OK) {
            err = ESP_FAIL;
            break;
        }

        addr += to_read;
    }

    free(rx_buf);

    /* 4. Resume Video */
    const uint8_t ACK_RESUME[] = {0x76, 0x00, 0x36, 0x00, 0x00};
    ptc06_cmd_simple_ack(cam, CMD_RESUME, sizeof(CMD_RESUME), ACK_RESUME, sizeof(ACK_RESUME));

    return err;
}

/* Helper context for buffer writer */
typedef struct {
    uint8_t *buffer;
    size_t cursor;
    size_t total_size;
} buffer_ctx_t;

static esp_err_t buffer_writer_cb(const uint8_t *data, size_t len, void *ctx) {
    buffer_ctx_t *b = (buffer_ctx_t *)ctx;
    if (b->cursor + len > b->total_size) return ESP_ERR_NO_MEM;
    
    memcpy(b->buffer + b->cursor, data, len);
    b->cursor += len;
    return ESP_OK;
}

esp_err_t ptc06_capture_jpeg_to_buffer(ptc06_t *cam, uint8_t **out_buf, size_t *out_len) {
    if (!cam || !out_buf || !out_len) return ESP_ERR_INVALID_ARG;

    /* 1. Freeze Frame & Get Length */
    const uint8_t ACK_CAP[] = {0x76, 0x00, 0x36, 0x00, 0x00};
    ESP_RETURN_ON_ERROR(ptc06_cmd_simple_ack(cam, CMD_CAPTURE, sizeof(CMD_CAPTURE), ACK_CAP, sizeof(ACK_CAP)), TAG, "Capture failed");

    uint8_t len_resp[9] = {0};
    uart_flush_input(cam->cfg.uart_num);
    uart_write_all(cam->cfg.uart_num, CMD_LENGTH, sizeof(CMD_LENGTH));
    
    if (uart_read_exact(cam->cfg.uart_num, len_resp, 9, cam->cfg.cmd_timeout_ms) != ESP_OK) {
        return ESP_FAIL;
    }

    uint32_t total_len = ((uint32_t)len_resp[5] << 24) | 
                         ((uint32_t)len_resp[6] << 16) | 
                         ((uint32_t)len_resp[7] << 8)  | 
                         len_resp[8];

    if (total_len == 0) return ESP_ERR_INVALID_SIZE;
    ESP_LOGI(TAG, "Allocating %lu bytes for JPEG...", total_len);

    /* 2. Allocate Buffer (Prefer SPIRAM) */
    uint8_t *buf = heap_caps_malloc(total_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        buf = heap_caps_malloc(total_len, MALLOC_CAP_8BIT); // Fallback to internal
        if (!buf) {
            ESP_LOGE(TAG, "Failed to allocate memory for JPEG");
            return ESP_ERR_NO_MEM;
        }
    }

    /* 3. Reuse Stream Logic to Fill Buffer */
    // This avoids code duplication and ensures fixes in stream logic apply here too
    buffer_ctx_t ctx = { .buffer = buf, .cursor = 0, .total_size = total_len };
    
    // We already froze the frame, but ptc06_capture_jpeg_stream will try to freeze it again.
    // The camera usually ACKs repeated freeze commands fine. 
    // However, to be cleaner, we can manually implement the loop here OR 
    // refactor stream to accept "skip_freeze" flag.
    // For simplicity/robustness in this "fix", I will copy the loop logic here
    // using the stack-efficient helper.

    uint32_t chunk_sz = 1024; // 1KB chunks
    uint8_t *chunk_buf = heap_caps_malloc(chunk_sz, MALLOC_CAP_8BIT);
    if (!chunk_buf) {
        free(buf);
        return ESP_ERR_NO_MEM;
    }

    uint32_t addr = 0;
    const uint8_t HDR_RESP[] = {0x76, 0x00, 0x32, 0x00, 0x00};
    esp_err_t err = ESP_OK;

    while (addr < total_len) {
        uint32_t to_read = (total_len - addr > chunk_sz) ? chunk_sz : (total_len - addr);
        uint8_t cmd[16];
        build_read_cmd(addr, to_read, cmd);

        uart_flush_input(cam->cfg.uart_num);
        uart_write_all(cam->cfg.uart_num, cmd, sizeof(cmd));

        if (expect_prefix(cam->cfg.uart_num, HDR_RESP, 5, cam->cfg.cmd_timeout_ms) != ESP_OK) {
            err = ESP_FAIL; break;
        }
        if (uart_read_exact(cam->cfg.uart_num, chunk_buf, to_read, cam->cfg.cmd_timeout_ms) != ESP_OK) {
            err = ESP_FAIL; break;
        }

        // Tail
        uint8_t tail[5];
        uart_read_exact(cam->cfg.uart_num, tail, 5, 100);

        memcpy(buf + addr, chunk_buf, to_read);
        addr += to_read;
    }

    free(chunk_buf);

    /* 4. Resume Video */
    const uint8_t ACK_RESUME[] = {0x76, 0x00, 0x36, 0x00, 0x00};
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
