#include "tm1637.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

static gpio_num_t g_dio, g_clk;

static inline void dly_us(int us) { esp_rom_delay_us(us); }
static inline void wr(gpio_num_t p, int v) { gpio_set_level(p, v); }
static inline void as_out(gpio_num_t p) { gpio_set_direction(p, GPIO_MODE_OUTPUT); }
static inline void as_in(gpio_num_t p)  { gpio_set_direction(p, GPIO_MODE_INPUT); gpio_set_pull_mode(p, GPIO_PULLUP_ONLY); }

// TM1637 bus primitives
static void start(void) { wr(g_dio,1); wr(g_clk,1); dly_us(5); wr(g_dio,0); dly_us(5); wr(g_clk,0); dly_us(5); }
static void stop(void)  { wr(g_clk,0); dly_us(5); wr(g_dio,0); dly_us(5); wr(g_clk,1); dly_us(5); wr(g_dio,1); dly_us(5); }

static int write_byte(uint8_t b) {
    for (int i = 0; i < 8; i++) {
        wr(g_clk, 0); dly_us(3);
        wr(g_dio, (b & 0x01)); dly_us(3);
        wr(g_clk, 1); dly_us(3);
        b >>= 1;
    }
    wr(g_clk, 0); dly_us(2);
    as_in(g_dio); dly_us(2);
    wr(g_clk, 1); dly_us(3);
    int ack = gpio_get_level(g_dio);   // 0 = ACK
    wr(g_clk, 0); dly_us(2);
    as_out(g_dio);
    return ack;
}

static const uint8_t DIGIT[10] = {
    0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F
};

static void show4(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3) {
    start(); write_byte(0x40); stop();          // data cmd: auto-increment
    start(); write_byte(0xC0);                  // addr 0
    write_byte(s0); write_byte(s1); write_byte(s2); write_byte(s3);
    stop();
}

void tm1637_init(gpio_num_t dio, gpio_num_t clk, int brightness_0_to_7)
{
    g_dio = dio; g_clk = clk;

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << g_dio) | (1ULL << g_clk),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io);

    wr(g_dio, 1);
    wr(g_clk, 1);

    // display on + brightness (0..7)
    start(); write_byte(0x88 | (brightness_0_to_7 & 0x07)); stop();

    // clear display
    show4(0x00,0x00,0x00,0x00);
}

void tm1637_show_hhmm(uint8_t hh, uint8_t mm, bool colon)
{
    uint8_t s0 = (hh >= 10) ? DIGIT[hh / 10] : 0x00;
    uint8_t s1 = DIGIT[hh % 10];
    uint8_t s2 = DIGIT[mm / 10];
    uint8_t s3 = DIGIT[mm % 10];
    if (colon) s1 |= 0x80;                 // colon bit on digit1
    show4(s0, s1, s2, s3);
}
