#include "fuelgauge.h"
#include <stdio.h>

// Function to read a register
uint16_t max17048_read_register(uint8_t reg) {
    uint8_t data[2];
    i2c_master_transmit_receive(fuel_gauge, &reg, 1, data, 2, -1);
    return (data[0] << 8) | data[1];
}

// Put MAX17048 into sleep mode
void max17048_sleep() {
    uint16_t config = max17048_read_register(0x0D);
    config |= (1 << 7);  // Set sleep bit
    uint8_t write_buf[3] = {0x0D, config >> 8, config & 0xFF};
    i2c_master_transmit(fuel_gauge, write_buf, 3, -1);
}

// Wake and read battery data
void max17048_wake_and_read(float *voltage, float *soc, float *rate) {
    uint16_t raw_voltage = max17048_read_register(0x02);
    uint16_t raw_soc = max17048_read_register(0x04);
    int16_t raw_rate = (int16_t)max17048_read_register(0x06);  // Ensure signed interpretation

    // Convert raw values to readable format
    *voltage = raw_voltage * 0.078125 / 1000;
    *soc = raw_soc * 0.00390625;
    *rate = raw_rate * 0.078125;
}
