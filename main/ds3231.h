#pragma once
#include <time.h>
#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_port_t port;      // e.g. I2C_NUM_0
    gpio_num_t sda;       // e.g. GPIO_NUM_21
    gpio_num_t scl;       // e.g. GPIO_NUM_22
    uint32_t   clk_hz;    // e.g. 100000 or 400000
} ds3231_config_t;

// Init DS3231 + install I2C driver on the given port (idempotent)
esp_err_t ds3231_init(const ds3231_config_t *cfg);

// Read RTC time into struct tm (interpreted as local time; set TZ before use)
esp_err_t ds3231_get_time(struct tm *out_tm);

// Write struct tm (local time) into RTC (24h mode)
esp_err_t ds3231_set_time(const struct tm *in_tm);

// Optional: read on-chip temperature (°C)
esp_err_t ds3231_get_temperature(float *out_c);

#ifdef __cplusplus
}
#endif
