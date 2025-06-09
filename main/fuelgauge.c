#include "fuelgauge.h"
#include <stdio.h>

i2c_master_dev_handle_t fuel_gauge;  // Define global variable

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
void max17048_wake_and_read() {
    uint16_t voltage = max17048_read_register(0x02);
    uint16_t soc = max17048_read_register(0x04);
    uint16_t rate = max17048_read_register(0x06);

    printf("Voltage: %.2f V\n", voltage * 0.078125 / 1000);
    printf("State of Charge: %.2f %%\n", soc * 0.00390625);
    printf("Discharge Rate: %.2f mV/h\n", (int16_t)rate * 0.078125);
}
