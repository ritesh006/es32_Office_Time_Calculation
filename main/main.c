// main.c — ESP-IDF v5.3.x
// SoftAP "check-in": phone connects -> (relearn MAC if needed) -> start 9:15 countdown -> delayed deauth.
// Timebase: DS3231 (I2C). Display: TM1637 (HH:MM). State: NVS.
// Fixes:
//  - NVS loads use temps (no volatile pointer warnings)
//  - Deauth by AID (IDF v5.3 API), not MAC
//  - No duplicate globals; safe printf formats

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "driver/i2c.h"

#include "tm1637.h"
#include "ds3231.h"

// ================= USER CONFIG =================
#define SOFTAP_SSID        "ESP32-Timekeeper"
#define SOFTAP_PASS        "timekeeper123"     // >=8 chars for WPA2
#define SOFTAP_CHANNEL     6
#define SOFTAP_MAX_CONN    2

// Deauth policy (choose ONE)
#define AUTO_DEAUTH_ON_FIRST_CONNECT  1       // deauth only on first connect of the day
// #define AUTO_DEAUTH_ALWAYS         1       // deauth on every connect (may cause loops)

// Relearn MAC on first connect of the day if it changed (handles phone's private MAC)
#define RELEARN_MAC_DAILY  1

// Delay before deauth so phone marks AP join as successful (helps auto-join later)
#define DEAUTH_DELAY_MS    4000               // 4 seconds

// TM1637 pins/brightness
#define TM_DIO_PIN         GPIO_NUM_16
#define TM_CLK_PIN         GPIO_NUM_17
#define TM_BRIGHTNESS      7

// DS3231 I2C
#define I2C_PORT           I2C_NUM_0
#define I2C_SDA            GPIO_NUM_21
#define I2C_SCL            GPIO_NUM_22
#define I2C_FREQ_HZ        400000

// Work target: 9h15m
#define DAILY_TARGET_SEC   (9*3600 + 15*60)   // 33300

// NVS storage
#define NVS_NS             "tk"
#define NVS_KEY_DAY        "day"              // uint32 (yyyymmdd)
#define NVS_KEY_REM        "rem"              // int32  (remaining seconds)
#define NVS_KEY_STARTED    "start"            // u8
#define NVS_KEY_MAC        "mac"              // blob(6)
#define NVS_KEY_HAVE_MAC   "hmac"             // u8

static const char *TAG = "timekeeper";

// ================ STATE ================
static volatile bool     s_started    = false;
static volatile int32_t  s_remaining  = DAILY_TARGET_SEC;
static volatile uint32_t s_day_key    = 0;     // yyyymmdd
static uint8_t           s_phone_mac[6] = {0};
static volatile bool     s_have_mac   = false;
static volatile bool     s_rtc_ok     = false;

static time_t            s_last_save_epoch = 0;

// Deauth timer state (v5.3: deauth by AID)
static esp_timer_handle_t s_deauth_timer  = NULL;
static volatile bool       s_deauth_pending = false;
static uint16_t            s_deauth_aid = 0;       // AID to deauth
static uint8_t             s_deauth_mac[6];        // just for logging

// ================ HELPERS ================
static inline uint32_t day_key_from_tm(const struct tm *t) {
    return (uint32_t)((t->tm_year + 1900) * 10000 + (t->tm_mon + 1) * 100 + t->tm_mday);
}
static inline time_t tm_local_to_epoch(struct tm tlocal) { return mktime(&tlocal); }

static void set_system_time_from_tm(const struct tm *t_local) {
    struct tm tmp = *t_local;
    struct timeval tv = { .tv_sec = mktime(&tmp), .tv_usec = 0 };
    settimeofday(&tv, NULL);
}

static void i2c_scan(i2c_port_t port) {
    printf("\n[I2C] scanning...\n");
    for (int addr = 0x03; addr <= 0x77; ++addr) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t r = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (r == ESP_OK) printf("  FOUND: 0x%02X\n", addr);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    printf("[I2C] scan done.\n\n");
}

static void print_mac(const char *prefix, const uint8_t mac[6]) {
    ESP_LOGI(TAG, "%s %02X:%02X:%02X:%02X:%02X:%02X",
             prefix,
             (unsigned int)mac[0], (unsigned int)mac[1], (unsigned int)mac[2],
             (unsigned int)mac[3], (unsigned int)mac[4], (unsigned int)mac[5]);
}

// ================ NVS ================
static void nvs_save_state(void) {
    time_t now; time(&now);
    if (now - s_last_save_epoch < 60) return; // <= 1/min
    s_last_save_epoch = now;

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    (void)nvs_set_u32(h, NVS_KEY_DAY, s_day_key);
    (void)nvs_set_i32(h, NVS_KEY_REM, s_remaining);
    (void)nvs_set_u8(h,  NVS_KEY_STARTED, s_started ? 1 : 0);
    (void)nvs_set_u8(h,  NVS_KEY_HAVE_MAC, s_have_mac ? 1 : 0);
    if (s_have_mac) (void)nvs_set_blob(h, NVS_KEY_MAC, s_phone_mac, 6);
    (void)nvs_commit(h);
    nvs_close(h);
}

static void nvs_save_state_immediate(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    (void)nvs_set_u32(h, NVS_KEY_DAY, s_day_key);
    (void)nvs_set_i32(h, NVS_KEY_REM, s_remaining);
    (void)nvs_set_u8(h,  NVS_KEY_STARTED, s_started ? 1 : 0);
    (void)nvs_set_u8(h,  NVS_KEY_HAVE_MAC, s_have_mac ? 1 : 0);
    if (s_have_mac) (void)nvs_set_blob(h, NVS_KEY_MAC, s_phone_mac, 6);
    (void)nvs_commit(h);
    nvs_close(h);
    s_last_save_epoch = 0;
}

static void nvs_load_state(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;

    uint8_t b = 0;
    uint32_t dk;
    int32_t rem;
    size_t len = 6;

    if (nvs_get_u32(h, NVS_KEY_DAY, &dk) == ESP_OK) s_day_key = dk;
    if (nvs_get_i32(h, NVS_KEY_REM, &rem) == ESP_OK) s_remaining = rem;
    if (nvs_get_u8(h, NVS_KEY_STARTED, &b) == ESP_OK) s_started = (b != 0);
    if (nvs_get_u8(h, NVS_KEY_HAVE_MAC, &b) == ESP_OK) s_have_mac = (b != 0);
    if (s_have_mac && nvs_get_blob(h, NVS_KEY_MAC, s_phone_mac, &len) != ESP_OK) {
        s_have_mac = false;
        memset(s_phone_mac, 0, 6);
    }
    nvs_close(h);
}

// ================ Deauth timer ================
static void deauth_timer_cb(void *arg) {
    if (s_deauth_pending && s_deauth_aid != 0) {
        esp_err_t r = esp_wifi_deauth_sta(s_deauth_aid); // IDF v5.3: by AID
        s_deauth_pending = false;
        if (r != ESP_OK) {
            ESP_LOGW(TAG, "deauth (AID=%u) failed: %s", (unsigned)s_deauth_aid, esp_err_to_name(r));
        } else {
            print_mac("Deauth sent (delayed) to:", s_deauth_mac);
        }
        s_deauth_aid = 0;
    }
}

// ================ Wi-Fi SoftAP ================
static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *ev = (wifi_event_ap_staconnected_t*)data;
        print_mac("STA connected:", ev->mac);
        ESP_LOGI(TAG, "STA AID=%u", (unsigned)ev->aid);

        bool mac_matches = s_have_mac && (memcmp(s_phone_mac, ev->mac, 6) == 0);
        bool can_relearn =
        #ifdef RELEARN_MAC_DAILY
            (!s_started);   // only before first check-in of the day
        #else
            false;
        #endif

        bool accept_as_phone = (!s_have_mac) || mac_matches || can_relearn;

        if (accept_as_phone) {
            if (!s_have_mac || (!mac_matches && can_relearn)) {
                memcpy((void*)s_phone_mac, ev->mac, 6);
                s_have_mac = true;
                print_mac("Phone MAC set/updated to:", s_phone_mac);
                nvs_save_state_immediate();
            }

            bool should_deauth = false;
            #ifdef AUTO_DEAUTH_ALWAYS
                should_deauth = true;
            #endif
            #ifdef AUTO_DEAUTH_ON_FIRST_CONNECT
                if (!s_started) should_deauth = true;
            #endif

            if (!s_started) {
                s_started = true;
                ESP_LOGI(TAG, "Checked in: starting today's countdown");
                nvs_save_state_immediate();
            } else {
                ESP_LOGI(TAG, "Already started today");
            }

            if (should_deauth && s_deauth_timer) {
                memcpy(s_deauth_mac, ev->mac, 6);
                s_deauth_aid = ev->aid;           // <-- capture AID for deauth
                s_deauth_pending = true;
                (void)esp_timer_stop(s_deauth_timer);
                ESP_LOGI(TAG, "Scheduling deauth in %d ms (AID=%u)", DEAUTH_DELAY_MS, (unsigned)s_deauth_aid);
                ESP_ERROR_CHECK(esp_timer_start_once(s_deauth_timer, (uint64_t)DEAUTH_DELAY_MS * 1000));
            } else {
                ESP_LOGI(TAG, "Deauth not scheduled (policy/state)");
            }
        } else {
            ESP_LOGW(TAG, "Unknown device ignored (stored MAC exists and does not match; not first connect of day)");
        }
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *ev = (wifi_event_ap_stadisconnected_t*)data;
        print_mac("STA disconnected:", ev->mac);
        ESP_LOGI(TAG, "STA AID=%u", (unsigned)ev->aid);
        if (s_deauth_timer) (void)esp_timer_stop(s_deauth_timer);
        s_deauth_pending = false;
        s_deauth_aid = 0;
    }
}

static void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t ap = {0};
    strncpy((char*)ap.ap.ssid, SOFTAP_SSID, sizeof(ap.ap.ssid)-1);
    strncpy((char*)ap.ap.password, SOFTAP_PASS, sizeof(ap.ap.password)-1);
    ap.ap.channel        = SOFTAP_CHANNEL;
    ap.ap.max_connection = SOFTAP_MAX_CONN;
    ap.ap.authmode       = (strlen(SOFTAP_PASS) >= 8) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    ap.ap.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP started: SSID=%s, PASS=%s, CH=%d", SOFTAP_SSID, SOFTAP_PASS, SOFTAP_CHANNEL);
}

// ================ App ================
void app_main(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
    ESP_ERROR_CHECK(nvs_flash_init());

    // IST (UTC+5:30). POSIX sign inverted.
    setenv("TZ", "IST-5:30", 1);
    tzset();

    // DS3231 init
    ds3231_config_t rtc = { .port=I2C_PORT, .sda=I2C_SDA, .scl=I2C_SCL, .clk_hz=I2C_FREQ_HZ };
    if (ds3231_init(&rtc) == ESP_OK) {
        s_rtc_ok = true;
        i2c_scan(I2C_PORT);
        struct tm t = {0};
        if (ds3231_get_time(&t) == ESP_OK) {
            printf("RTC @ boot: %04d-%02d-%02d %02d:%02d:%02d\n",
                   t.tm_year+1900, t.tm_mon+1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
            set_system_time_from_tm(&t);
        } else {
            ESP_LOGW(TAG, "RTC read failed @ boot");
        }
    } else {
        ESP_LOGW(TAG, "RTC init failed");
    }

    // Load persisted state
    nvs_load_state();

    // Establish today's key & handle day reset if needed
    struct tm now_tm = {0};
    time_t now_epoch;
    if (s_rtc_ok && ds3231_get_time(&now_tm) == ESP_OK) {
        uint32_t today = day_key_from_tm(&now_tm);
        if (s_day_key != today) {
            s_day_key   = today;
            s_started   = false;
            s_remaining = DAILY_TARGET_SEC;
            ESP_LOGI(TAG, "New day %" PRIu32 " - reset to 9:15", s_day_key);
            nvs_save_state_immediate();
        }
    } else {
        time(&now_epoch); localtime_r(&now_epoch, &now_tm);
        s_day_key = day_key_from_tm(&now_tm);
    }

    // Create deauth timer BEFORE starting AP
    {
        esp_timer_create_args_t targs = {
            .callback = deauth_timer_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "deauth"
        };
        ESP_ERROR_CHECK(esp_timer_create(&targs, &s_deauth_timer));
    }

    // Bring up SoftAP
    wifi_init_softap();

    // TM1637 init
    tm1637_init(TM_DIO_PIN, TM_CLK_PIN, TM_BRIGHTNESS);

    // Main loop — drive display & countdown
    time_t last_epoch = tm_local_to_epoch(now_tm);
    while (1) {
        struct tm t = {0};
        if (s_rtc_ok && ds3231_get_time(&t) == ESP_OK) {
            time_t epoch = mktime(&t);

            // Day boundary check (IST)
            uint32_t today = day_key_from_tm(&t);
            if (today != s_day_key) {
                s_day_key   = today;
                s_started   = false;
                s_remaining = DAILY_TARGET_SEC;
                ESP_LOGI(TAG, "New day %" PRIu32 " - reset to 9:15", s_day_key);
                nvs_save_state_immediate();
            }

            // Decrement by elapsed seconds (robust to delays)
            time_t delta = epoch - last_epoch;
            if (delta < 0) delta = 0;
            if (delta > 60) delta = 60;
            if (s_started && s_remaining > 0) {
                int32_t dec = (int32_t)delta;
                if (dec > s_remaining) dec = s_remaining;
                s_remaining -= dec;
                if (dec > 0 && (s_remaining % 60 == 0)) {
                    nvs_save_state();
                }
            }
            last_epoch = epoch;

            // Display remaining on TM1637 (HH:MM, blink colon)
            int rem = s_remaining; if (rem < 0) rem = 0;
            int rh = rem / 3600;
            int rm = (rem % 3600) / 60;
            bool colon = (t.tm_sec % 2) == 0;
            if (rh > 99) rh = 99;
            tm1637_show_hhmm((uint8_t)rh, (uint8_t)rm, colon);

            // UART single-line
            char timebuf[64];
            strftime(timebuf, sizeof(timebuf), "%I:%M:%S %p %d-%m-%Y IST", &t);
            const char *state = (s_remaining == 0) ? "DONE" : (s_started ? "RUN " : "WAIT");
            printf("\r\x1b[K%s | Rem %02d:%02d | %s", timebuf, rh, rm, state);
        } else {
            tm1637_show_hhmm(0, 0, false);
            printf("\r\x1b[KRTC read failed...");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
