#pragma once
#include "driver/i2c.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  DPS310_RATE_1HZ   = 0,
  DPS310_RATE_2HZ   = 1,
  DPS310_RATE_4HZ   = 2,
  DPS310_RATE_8HZ   = 3,
  DPS310_RATE_16HZ  = 4,
  DPS310_RATE_32HZ  = 5,
  DPS310_RATE_64HZ  = 6,
  DPS310_RATE_128HZ = 7,
} dps310_rate_t;

typedef enum {
  DPS310_OSR_1   = 0,
  DPS310_OSR_2   = 1,
  DPS310_OSR_4   = 2,
  DPS310_OSR_8   = 3,
  DPS310_OSR_16  = 4,
  DPS310_OSR_32  = 5,
  DPS310_OSR_64  = 6,
  DPS310_OSR_128 = 7,
} dps310_osr_t;

typedef enum {
  DPS310_MODE_IDLE    = 0b000,
  DPS310_MODE_ONE_P   = 0b001,
  DPS310_MODE_ONE_T   = 0b010,
  DPS310_MODE_CONT_P  = 0b101,
  DPS310_MODE_CONT_T  = 0b110,
  DPS310_MODE_CONT_PT = 0b111,
} dps310_mode_t;

typedef struct {
  i2c_port_t   i2c_port;
  uint8_t      i2c_addr;     // 0x77 (SDO=VDD) or 0x76 (SDO=GND)

  // calibration coeffs
  int16_t  c0, c1, c01, c11, c20, c21, c30;
  int32_t  c00, c10;

  // scaling by oversample
  int32_t  kT, kP;
  dps310_osr_t osr_t, osr_p;

  // true when coef source says "external sensor"
  bool     temp_ext;
} dps310_t;

esp_err_t dps310_init(dps310_t *dev, i2c_port_t port, uint8_t addr);
esp_err_t dps310_config(dps310_t *dev,
                        dps310_rate_t rate_t, dps310_osr_t osr_t,
                        dps310_rate_t rate_p, dps310_osr_t osr_p,
                        dps310_mode_t mode);
esp_err_t dps310_read(dps310_t *dev, float *temperature_c, float *pressure_hpa);

#ifdef __cplusplus
}
#endif
