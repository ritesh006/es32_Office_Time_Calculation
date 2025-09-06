# esp32_Seven_Sement_Clock

The code lives on the **feature branch**:
https://github.com/ritesh006/esp32_Seven_Sement_Clock/tree/feature/clock-ntp-tm1637
# ESP32 Clock — UART (12-hour) + TM1637 (HH:MM, blinking colon) via NTP (IST)

This branch turns an ESP32 into a tidy wall/desk clock:

- Connects to Wi-Fi (STA) and syncs time via **SNTP** in **immediate** mode (no smooth drift).
- Waits for sync before showing anything (so you never see a wrong time).
- Prints **12-hour time** with seconds on **UART** (single-line refresh).
- Shows **HH:MM** on a **TM1637 4-digit** 7-segment with a **blinking colon**.

> Target timezone: **Asia/Kolkata (IST)**.

---

## Hardware

- **ESP32** DevKit (ESP32-WROOM/ESP32-DevKitC)
- **TM1637 4-digit** display module

### Wiring

| TM1637 Pin | ESP32 Pin | Notes                                                              |
|------------|-----------|--------------------------------------------------------------------|
| VCC        | 5V        | 5V for good brightness (3.3V also works on many modules)          |
| GND        | GND       | Common ground                                                      |
| DIO        | GPIO16    | Configurable in code                                               |
| CLK        | GPIO17    | Configurable in code                                               |

> TM1637 modules typically tolerate 3.3V logic even on 5V VCC (most have on-board resistors). If your module is unusual, power at 3.3V.

---

## What you’ll see

- **USB serial (UART0)** @ **115200 bps**: `hh:mm:ss AM/PM dd-mm-YYYY IST` (refreshed on one line).
- **TM1637**: `HH:MM` (12-hour). The **colon blinks** every second.

---

## Project Layout (relevant files)

```
main/
  ├─ main.c          // Wi-Fi STA + SNTP (IST) + UART + TM1637 update loop
  ├─ tm1637.c        // Minimal bit-banged TM1637 driver
  ├─ tm1637.h
  └─ CMakeLists.txt  // Lists main.c + tm1637.c
```

---

## Quick Start

### 1) Configure Wi-Fi and display pins

Edit **`main/main.c`**:

```c
#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASS "WIFI_PASS"

#define TM_DIO_PIN     GPIO_NUM_16
#define TM_CLK_PIN     GPIO_NUM_17
#define TM_BRIGHTNESS  7   // 0..7
```

### 2) Build & Flash (VS Code ESP-IDF extension or CLI)

**VS Code (recommended):**
- `ESP-IDF: Full Clean` → `ESP-IDF: Build` → `ESP-IDF: Flash` → `ESP-IDF: Monitor`

**CLI:**
```bash
idf.py set-target esp32
idf.py fullclean
idf.py build flash monitor
```

> Serial monitor default: **115200** baud.

---

## Important SDK settings

Open **menuconfig** (VS Code → *SDK Configuration Editor*):

1. **Event loop stack** (prevents `sys_evt` stack overflow):
   - `Component config → Event loop library → Event loop task stack size` → **4096** (or **6144** if needed)

2. **Flash size** (fixes “Detected 4096k larger than header 2048k”):
   - `Serial flasher config → Flash size` → **4 MB (4096 kB)`
   - Then `Full Clean` → Build → Flash  
   - If still mismatched once, do `ESP-IDF: Erase Flash` and flash again

> You can persist defaults via an optional `sdkconfig.defaults`:
> ```
> CONFIG_ESP_EVENT_LOOP_TASK_STACK_SIZE=4096
> CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y
> ```

---

## How time sync works (and why it looks instant)

- SNTP servers: `pool.ntp.org`, `time.cloudflare.com`, `time.google.com`
- **Immediate mode** is enabled → the system clock is set at once (no slow slewing).
- The app **waits** for `SNTP_SYNC_STATUS_COMPLETED` before it prints/displays time.

---

## Customization

- **24-hour display** (TM1637): change the hour fed into the TM1637 call:
  ```c
  // Instead of h12, use ti.tm_hour (00..23)
  tm1637_show_hhmm((uint8_t)ti.tm_hour, (uint8_t)ti.tm_min, colon);
  ```
- **Quieter logs** after sync (keeps UART line clean):
  ```c
  // after time sync OK
  esp_log_level_set("*", ESP_LOG_WARN);
  ```
- **Extra NTP sources**: add more `esp_sntp_setservername(index, "host")`.

---

## Troubleshooting

- **Access denied / COM port busy**:
  - Close other serial apps/monitors (Arduino, PuTTY, prior Monitor tabs).
  - Unplug/replug USB. Reopen **Monitor** with only one instance.

- **Stack overflow in `sys_evt` or event loop**:
  - Increase **Event loop task stack size** to **4096–6144** in menuconfig.

- **Flash size mismatch**:
  - Set **Flash size = 4 MB** in menuconfig, `Full Clean`, Build, Flash.  
  - If persistent once, run **Erase Flash** then Flash.

- **NTP doesn’t sync** (stuck on “Waiting for NTP…”):
  - Ensure your Wi-Fi has internet and UDP/123 isn’t blocked.
  - Try a different AP or mobile hotspot temporarily to verify.

- **Python requirements error** (ESP-IDF tools):
  - Run **`install.bat`** for your ESP-IDF version or VS Code command  
    **“ESP-IDF: Install ESP-IDF Python Packages”** targeting v5.3.1.
---

**License:** same as repository’s root (or add one here if needed).
