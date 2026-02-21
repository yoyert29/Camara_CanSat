#include "dps310.h"
#include "esp_check.h"
#include "esp_log.h"

#define TAG "DPS310"

// Registers
#define DPS310_REG_PRSB2         0x00
#define DPS310_REG_TMPB2         0x03
#define DPS310_REG_PRSCFG        0x06
#define DPS310_REG_TMPCFG        0x07
#define DPS310_REG_MEASCFG       0x08
#define DPS310_REG_CFGREG        0x09
#define DPS310_REG_RESET         0x0C
#define DPS310_REG_PRODREVID     0x0D
#define DPS310_REG_COEF          0x10  // 18 bytes
#define DPS310_REG_TMP_COEF_SRCE 0x28

// default I2C addr if none provided
#define DPS310_ADDR_DEFAULT      0x77

// ---------- I2C helpers ----------
static esp_err_t wr8(dps310_t *dev, uint8_t reg, uint8_t val){
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev->i2c_addr<<1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, val, true);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(dev->i2c_port, cmd, 1000/portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}

static esp_err_t rd(dps310_t *dev, uint8_t reg, uint8_t *buf, size_t n){
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev->i2c_addr<<1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev->i2c_addr<<1) | I2C_MASTER_READ, true);
  if (n > 1) i2c_master_read(cmd, buf, n-1, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, buf+n-1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(dev->i2c_port, cmd, 1000/portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}

static int32_t sx24(int32_t x){ if (x & 0x800000) x -= 1<<24; return x; }
static int32_t sx20(int32_t x){ if (x & 0x80000)  x -= 1<<20; return x; }
static int16_t sx12(int16_t x){ if (x & 0x0800)  x -= 1<<12; return x; }
static int16_t sx16(int16_t x){ if (x & 0x8000)  x -= 1<<16; return x; }

// Wait helper: poll MEASCFG until (mask bits) == expect
static esp_err_t wait_bits(dps310_t *dev, uint8_t mask, uint8_t expect, int timeout_ms){
  TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
  while (xTaskGetTickCount() < deadline) {
    uint8_t m=0;
    esp_err_t e = rd(dev, DPS310_REG_MEASCFG, &m, 1);
    if (e != ESP_OK) return e;
    if ((m & mask) == expect) return ESP_OK;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return ESP_ERR_TIMEOUT;
}

// Robust scale mapping (don’t assume enum is 0..7)
static int32_t scale_from_osr(dps310_osr_t osr){
  switch(osr){
    case DPS310_OSR_1:   return 524288;   // 1x
    case DPS310_OSR_2:   return 1572864;  // 2x
    case DPS310_OSR_4:   return 3670016;  // 4x
    case DPS310_OSR_8:   return 7864320;  // 8x
    case DPS310_OSR_16:  return 253952;   // 16x
    case DPS310_OSR_32:  return 516096;   // 32x
    case DPS310_OSR_64:  return 1040384;  // 64x
    case DPS310_OSR_128: return 2088960;  // 128x
    default:              return 524288;
  }
}

static esp_err_t read_coeffs(dps310_t *dev){
  uint8_t b[18];
  ESP_RETURN_ON_ERROR(rd(dev, DPS310_REG_COEF, b, sizeof b), TAG, "coef read");

  int16_t c0  = sx12((int16_t)((b[0] << 4) | (b[1] >> 4)));
  int16_t c1  = sx12((int16_t)(((b[1] & 0x0F) << 8) | b[2]));
  int32_t c00 = sx20((int32_t)((b[3] << 12) | (b[4] << 4) | (b[5] >> 4)));
  int32_t c10 = sx20((int32_t)(((b[5] & 0x0F) << 16) | (b[6] << 8) | b[7]));
  int16_t c01 = sx16((int16_t)((b[8] << 8) | b[9]));
  int16_t c11 = sx16((int16_t)((b[10] << 8) | b[11]));
  int16_t c20 = sx16((int16_t)((b[12] << 8) | b[13]));
  int16_t c21 = sx16((int16_t)((b[14] << 8) | b[15]));
  int16_t c30 = sx16((int16_t)((b[16] << 8) | b[17]));

  dev->c0=c0; dev->c1=c1; dev->c00=c00; dev->c10=c10;
  dev->c01=c01; dev->c11=c11; dev->c20=c20; dev->c21=c21; dev->c30=c30;
  ESP_LOGI(TAG,"coeff ok (c0=%d c1=%d c00=%ld c10=%ld)", c0, c1, (long)c00, (long)c10);
  return ESP_OK;
}

esp_err_t dps310_init(dps310_t *dev, i2c_port_t port, uint8_t addr){
  dev->i2c_port = port;
  dev->i2c_addr = addr ? addr : DPS310_ADDR_DEFAULT;

  // soft reset
  ESP_RETURN_ON_ERROR(wr8(dev, DPS310_REG_RESET, 0x09), TAG, "reset");
  vTaskDelay(pdMS_TO_TICKS(10));

  // Wait for SENSOR_RDY (bit6) and COEF_RDY (bit7)
  ESP_RETURN_ON_ERROR(wait_bits(dev, 0xC0, 0xC0, 200), TAG, "sensor/coef not ready");

  uint8_t id=0;
  ESP_RETURN_ON_ERROR(rd(dev, DPS310_REG_PRODREVID, &id, 1), TAG, "prod id");
  ESP_LOGI(TAG, "prod/rev id=0x%02X (expect 0x10)", id);

  uint8_t src=0;
  ESP_RETURN_ON_ERROR(rd(dev, DPS310_REG_TMP_COEF_SRCE, &src, 1), TAG, "coef src");
  dev->temp_ext = (src & 0x80) != 0;
  ESP_LOGI(TAG, "TMP coef source: %s", dev->temp_ext ? "EXTERNAL" : "INTERNAL");

  // read coeffs AFTER COEF_RDY
  ESP_RETURN_ON_ERROR(read_coeffs(dev), TAG, "coeff");

  // defaults until user configures
  dev->osr_t = DPS310_OSR_16;
  dev->osr_p = DPS310_OSR_16;
  dev->kT = scale_from_osr(dev->osr_t);
  dev->kP = scale_from_osr(dev->osr_p);
  return ESP_OK;
}

esp_err_t dps310_config(dps310_t *dev,
                        dps310_rate_t rate_t, dps310_osr_t osr_t,
                        dps310_rate_t rate_p, dps310_osr_t osr_p,
                        dps310_mode_t mode)
{
  // PRSCFG: [7:4]=rate, [3:0]=osr
  ESP_RETURN_ON_ERROR(wr8(dev, DPS310_REG_PRSCFG,
                          ((rate_p & 0x0F)<<4) | (osr_p & 0x0F)), TAG, "prscfg");

  // TMPCFG: [7]=TMP_EXT, [6:4]=rate, [3:0]=osr -> match coef source
  uint8_t tmpcfg = ((rate_t & 0x0F)<<4) | (osr_t & 0x0F);
  if (dev->temp_ext) tmpcfg |= 0x80;
  ESP_RETURN_ON_ERROR(wr8(dev, DPS310_REG_TMPCFG, tmpcfg), TAG, "tmpcfg");

  // CFGREG: shift enable for OSR > 8
  uint8_t cfg = 0;
  if (osr_p > DPS310_OSR_8) cfg |= 0x04; // P_SHIFT_EN
  if (osr_t > DPS310_OSR_8) cfg |= 0x08; // T_SHIFT_EN
  ESP_RETURN_ON_ERROR(wr8(dev, DPS310_REG_CFGREG, cfg), TAG, "cfgreg");

  // MEASCFG: mode
  ESP_RETURN_ON_ERROR(wr8(dev, DPS310_REG_MEASCFG, (uint8_t)mode), TAG, "meascfg");

  dev->osr_t = osr_t; dev->osr_p = osr_p;
  dev->kT = scale_from_osr(dev->osr_t);
  dev->kP = scale_from_osr(dev->osr_p);
  return ESP_OK;
}

static esp_err_t read_raw_tp(dps310_t *dev, int32_t *raw_t, int32_t *raw_p){
  uint8_t b[3];
  ESP_RETURN_ON_ERROR(rd(dev, DPS310_REG_PRSB2, b, 3), TAG, "raw P");
  int32_t p = sx24(((int32_t)b[0] << 16) | ((int32_t)b[1] << 8) | b[2]);
  ESP_RETURN_ON_ERROR(rd(dev, DPS310_REG_TMPB2, b, 3), TAG, "raw T");
  int32_t t = sx24(((int32_t)b[0] << 16) | ((int32_t)b[1] << 8) | b[2]);
  *raw_p = p; *raw_t = t;
  return ESP_OK;
}

esp_err_t dps310_read(dps310_t *dev, float *temperature_c, float *pressure_hpa){
  // Wait for both TMP_RDY (bit5) and PRS_RDY (bit4) up to ~200 ms
  ESP_RETURN_ON_ERROR(wait_bits(dev, 0x30, 0x30, 200), TAG, "data not ready");

  int32_t raw_t=0, raw_p=0;
  ESP_RETURN_ON_ERROR(read_raw_tp(dev, &raw_t, &raw_p), TAG, "raw rd");
  ESP_LOGD(TAG, "raw T=%ld raw P=%ld", (long)raw_t, (long)raw_p);

  float tr = (float)raw_t / (float)dev->kT;
  float pr = (float)raw_p / (float)dev->kP;

  float T = dev->c0 * 0.5f + dev->c1 * tr;
  float p = dev->c00 + pr * (dev->c10 + pr*(dev->c20 + pr*dev->c30))
            + tr * (dev->c01 + pr*(dev->c11 + pr*dev->c21));

  if (temperature_c) *temperature_c = T;
  if (pressure_hpa)  *pressure_hpa  = p / 100.0f; // Pa -> hPa
  return ESP_OK;
}
