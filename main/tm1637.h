
#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize display on given pins; brightness 0..7 (also turns display ON)
void tm1637_init(gpio_num_t dio, gpio_num_t clk, int brightness_0_to_7);

// Show HH:MM with optional blinking colon
void tm1637_show_hhmm(uint8_t hh, uint8_t mm, bool colon);

#ifdef __cplusplus
}
#endif
