#ifndef BME280_H
#define BME280_H

#include <stdint.h>   // For fixed-width integer types like uint8_t, int16_t, etc.
#include "esp_err.h"
#include "driver/i2c_master.h"

#define BME280_SENSOR_ADDR 0x76       // I2C address for the BME280 sensor

#define I2C_MASTER_SCL_IO 23          // GPIO for SCL
#define I2C_MASTER_SDA_IO 22          // GPIO for SDA
#define I2C_MASTER_NUM I2C_NUM_0      // I2C port number
#define I2C_MASTER_FREQ_HZ 100000     // I2C clock frequency (100kHz)
#define I2C_MASTER_TX_BUF_DISABLE 0   // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0   // I2C master doesn't need buffer
#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1
#define I2C_MASTER_TIMEOUT_MS 1000

extern i2c_master_bus_handle_t i2c_bus_handle;  // Global variable
extern i2c_master_dev_handle_t i2c_dev_handle;

// BME280 calibration parameters (stored in the sensor)
extern uint16_t dig_T1;
extern int16_t dig_T2, dig_T3;
extern uint16_t dig_P1;
extern int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
extern uint8_t dig_H1, dig_H3;
extern int16_t dig_H2, dig_H4, dig_H5, dig_H6;
extern int32_t t_fine;

esp_err_t bme280_read(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t bme280_write(uint8_t reg_addr, uint8_t data);

void bme280_init();
void bme280_read_calibration_data();

int32_t bme280_compensate_T(int32_t adc_T, int32_t *t_fine_out);
uint32_t bme280_compensate_P(int32_t adc_P, int32_t t_fine);
uint32_t bme280_compensate_H(int32_t adc_H);

#endif // BME280_H
