#ifdef __cplusplus
extern "C" {
#endif
#define TMP117_H

#include <stdint.h>



// TMP117 I2C Address
#define TMP117_ADDR 0x48 // Default I2C address of TMP117

// TMP117 Register Addresses
#define TMP117_TEMP_REG    0x00 // Temperature register
#define TMP117_CONFIG_REG  0x01 // Configuration register
#define TMP117_LOW_LIMIT   0x02 // Low limit register
#define TMP117_HIGH_LIMIT  0x03 // High limit register

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_FREQ_HZ 400000


// Function Prototypes

/**
 * @brief Initialize TMP117 sensor by configuring its registers.
 */
void tmp117_init(void);

/**
 * @brief Read the raw temperature value from the TMP117 sensor.
 *
 * @return Raw temperature value (16-bit signed integer).
 */
int16_t tmp117_read_raw(void);

/**
 * @brief Convert raw TMP117 temperature data to Celsius.
 *
 * @param raw_temp The raw temperature value read from TMP117.
 * @return Temperature in degrees Celsius as a float.
 */
float tmp117_compensate(int16_t raw_temp);


#ifdef __cplusplus
}

#endif // TMP117_H