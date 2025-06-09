#ifndef FUELGAUGE_H
#define FUELGAUGE_H

#include "driver/i2c_master.h"

extern i2c_master_dev_handle_t fuel_gauge;

// Function prototypes
uint16_t max17048_read_register(uint8_t reg);
void max17048_sleep();
void max17048_wake_and_read();

#endif