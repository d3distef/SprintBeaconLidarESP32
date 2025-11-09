# Lidar_Beacon – ESP32-S3 Zero (Single-Beacon Sprint Timer)

Self-contained sprint timer using **two LiDARs** (TF-Luna for finish detect, XT-S1 for background range/status), a **WS2812 8×8** display, and optional **OTA** + **BLE** control.

- Mode: **Standalone** (no ESP-NOW)  
- **BLE (NimBLE)**: write `{"arm":1}` to begin a randomized countdown (2–5 s)  
- **OTA on demand**: long-press button at boot (or during run)

---

## Features

- **Randomized start** (2–5 s) after `{"arm":1}` over BLE
- **Finish detection** via **TF-Luna** with debounce/hysteresis and a brief ignore window after start
- **XT-S1 Modbus polling** (background; paused during COUNTDOWN/RUNNING to keep timing clean)
- **8×8 WS2812** visual states:
  - Standby → rotating blue row  
  - Armed/Countdown → fire effect  
  - Running → green waves  
  - Finished → rainbow
- **OTA** (STA first, AP fallback) with a purple “breathing” LED while flashing
- **Button**: short press = restart; long press (≥2 s) = enter OTA

---

## Hardware

- **MCU**: ESP32-S3 Zero/Mini-class board (Arduino core)
- **LiDARs**:
  - **TF-Luna** (UART2 @ 115200) — finish detect
  - **XT-S1** (UART1 @ 115200, Modbus) — range/status
- **LEDs**: WS2812 8×8 on **GPIO10**
  - Level shift with **SN74AHCT125**
  - **330 Ω** series resistor on DIN
  - **≥1000 µF** bulk cap across LED 5 V ↔ GND
- **Power**: 5 V supply (USB-C breakout is fine). **Common ground** everywhere.

---

## Pins (GPIO numbers)

```c
// Button
static const int PIN_BTN  = 11;   // to GND, INPUT_PULLUP

// LEDs
static const int PIN_LEDS = 10;   // WS2812 DIN (use level shifter + 330 Ω)
static const int NUM_LEDS = 64;

// TF-Luna (finish) on UART2
static const int LUNA_RX = 13;    // MCU RX
static const int LUNA_TX = 12;    // MCU TX

// XT-S1 (status) on UART1 / Modbus
static const int XT_RX   = 44;    // MCU RX
static const int XT_TX   = 43;    // MCU TX
static const int LIDAR_BAUD = 115200;
```

> These are **GPIO identifiers** (not “Dx” labels).

---

## Wiring (quick)

```
5 V  ───────────────> ESP32 5V/VBUS + LED 5V
GND ───────────────> Common ground (ESP32, LED, both LiDARs)

WS2812 DIN ────────> GPIO10 via 74AHCT125 (5 V) + 330 Ω series
WS2812 +5/GND ────> 5 V rail (place ≥1000 µF cap at strip start)

TF-Luna  TX ──────> GPIO13 (LUNA_RX)
TF-Luna  RX ──────> GPIO12 (LUNA_TX)
TF-Luna  GND/5V ──> GND / 5 V

XT-S1    TX ──────> GPIO44 (XT_RX)
XT-S1    RX ──────> GPIO43 (XT_TX)
XT-S1    GND/5V ──> GND / 5 V

Button ───────────> GPIO11 ↔ GND  (INPUT_PULLUP)
```

---

## BLE

**Service UUID:** `b8c7f3f4-4b9f-4a5b-9c39-36c6b4c7e0a1`

**Characteristics:**
- **STATUS** (`…0b2`): READ | NOTIFY — JSON (phase, times, sensor values)
- **CONTROL** (`…0c3`): WRITE / WRITE_NR — send `{"arm":1}` to start
- **RANGE** (`…0d4`): READ | NOTIFY — `uint16` centimeters (`0xFFFF` if unknown)
- **SPRINT** (`…0f6`): READ | NOTIFY — `uint32` ms (LE)

**Arm sequence:**
1. Connect over BLE.  
2. Write `{"arm":1}` to **CONTROL** → device enters **ARMED**, randomizes 2–5 s countdown.  
3. At countdown expiry → **RUNNING**; TF-Luna detects finish → **SPRINT** notifies.

---

## OTA (on demand)

- **Hostname:** `Lidar_Beacon` (password `"U"` — change in code)
- **Enter OTA:**
  - **At boot:** hold the button **LOW** for ~250 ms during power-up  
  - **Anytime:** long press (≥2 s) to trigger Wi-Fi + OTA
- **Wi-Fi:** tries STA with `WIFI_SSID/PASS` from `secrets.h`, else AP fallback:  
  `LidarBeacon-<tail>` / password `BeaconSetup`
- **In OTA:** the 8×8 shows a **purple breathing** effect

---

## Build & Setup

### 1) Secrets

Create `secrets.h` next to the sketch:

```c
// secrets.h
const char* WIFI_SSID = "YourSSID";
const char* WIFI_PASS = "YourPass";
```

### 2) Arduino IDE

- Board: **ESP32S3 Dev Module** (or your S3 Zero/Mini equivalent)  
- Libraries: **FastLED**, **NimBLE-Arduino**, **ArduinoOTA** (bundled with ESP32 core)

### 3) Notes

- `FASTLED_RMT_MAX_CHANNELS 1` and `FASTLED_ESP32_FLASH_LOCK 1` are defined for ESP32-S3 stability.  
- XT-S1 Modbus polling is **paused during COUNTDOWN/RUNNING** to keep finish timing clean.

---

## Phases (firmware)

`STANDBY → ARMED (after {"arm":1}) → COUNTDOWN → RUNNING → FINISHED`  
(automatically returns to **STANDBY** after ~5 s)

---

## Troubleshooting

- **No finish:** verify TF-Luna wiring/baud and ensure the target passes close enough (drop ≥ 10 cm by default).  
- **LED flicker or resets:** ensure **≥1000 µF** at LED power input, short/thick 5 V wiring, and proper **AHCT** level shifting.  
- **OTA not appearing:** confirm long-press duration and valid Wi-Fi creds; check AP fallback SSID.

---

## License

MIT (update as desired)
