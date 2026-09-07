#pragma once

#include <stdbool.h>
#include <time.h>

#include "driver/gpio.h"

void ds1302_init(gpio_num_t clk_pin, gpio_num_t data_pin, gpio_num_t rst_pin);
bool ds1302_read_time(struct tm *local_time);
bool ds1302_write_time(const struct tm *local_time);
