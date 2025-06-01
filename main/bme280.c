#include "bme280.h"
#include "esp_log.h"

#define BME280_TAG "BME280"

// BME280 calibration parameters (stored in the sensor)
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t dig_H1, dig_H3;
int16_t dig_H2, dig_H4, dig_H5, dig_H6;
int32_t t_fine;

// Function to read from BME280
esp_err_t bme280_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Function to write to BME280
esp_err_t bme280_write(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// BME280 initialization sequence
void bme280_init() {
    // Write reset value to reset register (0xE0)
    bme280_write(0xE0, 0xB6);
    
    // Configure the sensor (you can adjust the config registers based on your needs)
    bme280_write(0xF2, 0x01); // Humidity oversampling x1
    bme280_write(0xF4, 0x27); // Temperature and pressure oversampling x1, mode = normal
    bme280_write(0xF5, 0xA0); // Standby time = 1000ms, IIR filter = x4
}

// Function to read calibration data from the BME280 sensor
void bme280_read_calibration_data() {
    uint8_t data[26];
    
    // Read temperature and pressure calibration values
    if (bme280_read(0x88, data, 24) != ESP_OK) {
        ESP_LOGE(BME280_TAG, "Failed to read calibration data");
        return;
    }

    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];

    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11] << 8) | data[10];
    dig_P4 = (data[13] << 8) | data[12];
    dig_P5 = (data[15] << 8) | data[14];
    dig_P6 = (data[17] << 8) | data[16];
    dig_P7 = (data[19] << 8) | data[18];
    dig_P8 = (data[21] << 8) | data[20];
    dig_P9 = (data[23] << 8) | data[22];

    // Read humidity calibration values
    if (bme280_read(0xA1, &dig_H1, 1) != ESP_OK) {
        ESP_LOGE(BME280_TAG, "Failed to read humidity calibration data");
        return;
    }
    if (bme280_read(0xE1, data, 7) != ESP_OK) {
        ESP_LOGE(BME280_TAG, "Failed to read humidity calibration data");
        return;
    }
    
    dig_H2 = (data[1] << 8) | data[0];
    dig_H3 = data[2];
    dig_H4 = (data[3] << 4) | (data[4] & 0x0F);
    dig_H5 = (data[5] << 4) | (data[4] >> 4);
    dig_H6 = data[6];
}

// Temperature compensation
int32_t bme280_compensate_T(int32_t adc_T, int32_t *t_fine_out) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    if (t_fine_out) {
        *t_fine_out = t_fine; // Assign the calculated t_fine to the location pointed to by t_fine_out
    }
    T = (t_fine * 5 + 128) >> 8;
    return T;  // Temperature in hundredths of a degree Celsius (e.g., 5123 is 51.23 °C)
}

uint32_t bme280_compensate_P(int32_t adc_P, int32_t t_fine) {
    int32_t var1, var2;
    uint32_t p;

    // Step 1: Calculate var1 based on t_fine
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;

    // Step 2: Calculate var2 based on var1 and calibration parameters
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)dig_P6;
    var2 += ((var1 * (int32_t)dig_P5) << 1);
    var2 = (var2 >> 2) + (((int32_t)dig_P4) << 16);

    // Step 3: Update var1 based on dig_P2 and dig_P3
    int32_t var1_part1 = ((var1 >> 2) * (var1 >> 2)) >> 13; // Calculate part1 of var1
    var1_part1 *= dig_P3; // Multiply by dig_P3
    var1_part1 >>= 3; // Right shift by 3
    
    int32_t var1_part2 = (int32_t)dig_P2 * var1; // Calculate part2 of var1
    var1_part2 >>= 1; // Right shift part2 by 1
    
    var1 = (var1_part1 + var1_part2) >> 18; // Combine part1 and part2 and right shift by 18

// Final adjustment for var1
var1 = (32768 + var1) * (int32_t)dig_P1 >> 15; // Final calculation for var1

    // Step 4: Check for division by zero
    if (var1 == 0) {
        return 0; // Avoid division by zero
    }

    // Step 5: Calculate the pressure value
    p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12)) * 3125);
    if (p < 0x80000000) {
        p = (p << 1) / (uint32_t)var1;
    } else {
        p = (p / (uint32_t)var1) * 2;
    }

    // Step 6: Final adjustments using dig_P9 and dig_P8
    var1 = (((int32_t)dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)dig_P8)) >> 13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + (int32_t)dig_P7) >> 4));

    // Convert from Pa to hPa
    return p; // Return pressure in hPa
}

// Humidity compensation
uint32_t bme280_compensate_H(int32_t adc_H) {
    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12); // Humidity in %RH (scaled by 1024)
}
