#include "ds3231.h"
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define DS3231_ADDR         0x68
#define REG_SECONDS         0x00
#define REG_MINUTES         0x01
#define REG_HOURS           0x02
#define REG_DAY             0x03
#define REG_DATE            0x04
#define REG_MONTH           0x05
#define REG_YEAR            0x06
#define REG_TEMP_MSB        0x11
#define REG_TEMP_LSB        0x12

static const char *TAG = "ds3231";
static i2c_port_t s_port = I2C_NUM_0;

static inline uint8_t bcd2bin(uint8_t v) { return (v & 0x0F) + 10 * ((v >> 4) & 0x0F); }
static inline uint8_t bin2bcd(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

esp_err_t ds3231_init(const ds3231_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    s_port = cfg->port;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = cfg->sda,
        .scl_io_num = cfg->scl,
        // Most DS3231 modules already have 3.3V pull-ups onboard:
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = cfg->clk_hz,
        .clk_flags = 0
    };

    esp_err_t err = i2c_param_config(s_port, &conf);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(s_port, I2C_MODE_MASTER, 0, 0, 0);
    if (err == ESP_ERR_INVALID_STATE) {
        // already installed → OK
        err = ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t ds3231_get_time(struct tm *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;

    uint8_t reg = REG_SECONDS;
    uint8_t b[7] = {0};
    esp_err_t err = i2c_master_write_read_device(
        s_port, DS3231_ADDR, &reg, 1, b, sizeof(b), pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read time failed: %s", esp_err_to_name(err));
        return err;
    }

    int sec = bcd2bin(b[0] & 0x7F);
    int min = bcd2bin(b[1] & 0x7F);

    uint8_t hr_reg = b[2];
    int hour;
    if (hr_reg & 0x40) { // 12h
        hour = bcd2bin(hr_reg & 0x1F);
        if (hour == 12) hour = 0;
        if (hr_reg & 0x20) hour += 12; // PM
    } else { // 24h
        hour = bcd2bin(hr_reg & 0x3F);
    }

    int mday = bcd2bin(b[REG_DATE - REG_SECONDS] & 0x3F);
    int mon  = bcd2bin(b[REG_MONTH - REG_SECONDS] & 0x1F) - 1;
    int year = bcd2bin(b[REG_YEAR  - REG_SECONDS]); // 00..99 -> 2000..2099
    int y1900 = (2000 + year) - 1900;

    memset(out, 0, sizeof(*out));
    out->tm_sec  = sec;
    out->tm_min  = min;
    out->tm_hour = hour;
    out->tm_mday = mday;
    out->tm_mon  = mon;
    out->tm_year = y1900;
    // tm_wday and others can be derived by mktime() if needed.

    return ESP_OK;
}

esp_err_t ds3231_set_time(const struct tm *in)
{
    if (!in) return ESP_ERR_INVALID_ARG;

    uint8_t w[8];
    w[0] = REG_SECONDS;
    w[1] = bin2bcd((uint8_t)in->tm_sec)  & 0x7F;
    w[2] = bin2bcd((uint8_t)in->tm_min)  & 0x7F;
    w[3] = bin2bcd((uint8_t)in->tm_hour) & 0x3F; // 24h mode (bit6=0)

    int wday = in->tm_wday;
    if (wday == 0) wday = 7; // DS3231 uses 1..7, where 1=Sun
    w[4] = bin2bcd((uint8_t)wday) & 0x07;

    w[5] = bin2bcd((uint8_t)in->tm_mday) & 0x3F;
    w[6] = bin2bcd((uint8_t)(in->tm_mon + 1)) & 0x1F;

    int y2000 = (in->tm_year + 1900) - 2000;
    if (y2000 < 0) {
        y2000 = 0;
    } else if (y2000 > 99) {
        y2000 = 99;
    }
    w[7] = bin2bcd((uint8_t)y2000);

    esp_err_t err = i2c_master_write_to_device(
        s_port, DS3231_ADDR, w, sizeof(w), pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write time failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t ds3231_get_temperature(float *out_c)
{
    if (!out_c) return ESP_ERR_INVALID_ARG;

    uint8_t reg = REG_TEMP_MSB;
    uint8_t b[2] = {0};
    esp_err_t err = i2c_master_write_read_device(
        s_port, DS3231_ADDR, &reg, 1, b, 2, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read temperature failed: %s", esp_err_to_name(err));
        return err;
    }

    // Temp: MSB is integer, high 2 bits of LSB are fractional (.25 steps)
    int8_t  msb = (int8_t)b[0];
    uint8_t lsb = b[1] >> 6;       // top 2 bits
    *out_c = (float)msb + (lsb * 0.25f);
    return ESP_OK;
}
