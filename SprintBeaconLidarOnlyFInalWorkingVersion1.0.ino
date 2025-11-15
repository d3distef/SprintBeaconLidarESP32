/*
  Lidar_Beacon – ESP32-S3 Zero (single-beacon sprint timer)
  - OTA hostname: "Lidar_Beacon"
  - BLE: same UUIDs; write {"arm":1} to start randomized (2–5s) countdown
  - TF-Luna finish on UART2 (RX=13, TX=12), XT-S1 status on UART1 (RX=44, TX=43)
  - WS2812 8x8 on GPIO10 (use 74AHCT125 level shifter + 330Ω + 1000µF)
  - Button on GPIO11: short<1s=reset, long>=2s=enter OTA
*/

#define FASTLED_RMT_MAX_CHANNELS 1   // keep RMT free/stable on ESP32-S3
#define FASTLED_ESP32_FLASH_LOCK 1   // safety with flash/OTA

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <NimBLEDevice.h>
#include <FastLED.h>
#include "secrets.h"  // WIFI_SSID, WIFI_PASS

// ===== Pins =====
static const int PIN_BTN  = 11;
static const int PIN_LEDS = 10;
static const int NUM_LEDS = 64;

HardwareSerial LUNA(2);   // TF-Luna (finish)
HardwareSerial XT(1);     // XT-S1   (status)
static const int LUNA_RX = 13;
static const int LUNA_TX = 12;
static const int XT_RX   = 44;
static const int XT_TX   = 43;
static const int LIDAR_BAUD = 115200;
// Clamp the TF-Luna background to 6 ft (183 cm)
static const uint16_t BASELINE_MAX_CM = 1830;

// ===== LEDs =====
CRGB leds[NUM_LEDS];

// ===== OTA =====
static const char* OTA_HOSTNAME = "Lidar_Beacon";
static const char* OTA_PASSWORD = "U";
static bool gOTA     = false;  // true once ArduinoOTA.begin() has been called
static bool bootOTA  = false;  // set true if button is held low during the 250ms boot window

// ===== BLE UUIDs (compat) =====
#define UUID_SERVICE        "b8c7f3f4-4b9f-4a5b-9c39-36c6b4c7e0a1"
#define UUID_CHAR_STATUS    "b8c7f3f4-4b9f-4a5b-9c39-36c6b4c7e0b2"
#define UUID_CHAR_CONTROL   "b8c7f3f4-4b9f-4a5b-9c39-36c6b4c7e0c3"
#define UUID_CHAR_RANGE     "b8c7f3f4-4b9f-4a5b-9c39-36c6b4c7e0d4"
#define UUID_CHAR_SPRINT    "b8c7f3f4-4b9f-4a5b-9c39-36c6b4c7e0f6"

static NimBLEServer*         bleServer   = nullptr;
static NimBLEService*        bleService  = nullptr;
static NimBLECharacteristic* chStatus    = nullptr;
static NimBLECharacteristic* chControl   = nullptr;
static NimBLECharacteristic* chRange     = nullptr;
static NimBLECharacteristic* chSprint    = nullptr;
static NimBLEAdvertising*    bleAdv      = nullptr;
static volatile bool         bleClientConnected = false;

// ===== State =====
enum class Phase : uint8_t { STANDBY, ARMED, COUNTDOWN, RUNNING, FINISHED };
static Phase   phase = Phase::STANDBY;
static uint32_t tPhaseStart = 0;
static uint32_t runStartMs  = 0;
static uint32_t sprintMs    = 0;
static uint32_t countdownTargetMs = 0;
static const uint32_t COUNTDOWN_MIN_MS = 2000;
static const uint32_t COUNTDOWN_MAX_MS = 5000;
static uint16_t finishTripMm = 1400;  // TF-Luna trip threshold

static uint32_t btnPressAt = 0;
static int32_t  luna_mm = -1;
static int32_t  xt_cm   = -1;
static volatile bool armRequested = false;

// ---- Finish detection (2 ft ~ 610 mm) with debounce & hysteresis ----
static const uint16_t FINISH_NEAR_MM = 61;  // <= this is a "pass by"
static const uint16_t FINISH_FAR_MM  = 80;  // must be above this before we allow a new "near"

static const uint8_t  BELOW_CONSEC_REQUIRED = 2;  // consecutive frames below NEAR to trigger
static const uint8_t  ABOVE_CONSEC_REQUIRED = 2;  // consecutive frames above FAR to re-arm

// Thresholds in CENTIMETERS
static const uint16_t MIN_ABS_CM   = 1;   // = 10 mm minimum absolute change
static const uint16_t MIN_DROP_CM  = 10;  // ≈ half a human body width (~46 cm / 18")
static uint16_t baseline_cm = 0;

static uint8_t belowConsec = 0;
static uint8_t aboveConsec = ABOVE_CONSEC_REQUIRED;  // start "armed"
static bool    nearArmed   = true;   // we only trigger when we're re-armed
static uint32_t runIgnoreUntilMs = 0; // small grace window after RUNNING starts


// ---- Modbus helpers for XT-S1 on UART1 (kept light and paused during timing) ----
static const uint8_t  MB_DEV_ID     = 0x01;
static const uint32_t MB_POLL_MS    = 250;   // light background load
static const uint32_t MB_RX_TIMEOUT = 12;    // fast-fail

static uint16_t mb_crc16(const uint8_t* d, size_t n){
  uint16_t c=0xFFFF;
  for(size_t i=0;i<n;i++){
    c ^= d[i];
    for(int b=0;b<8;b++) c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1);
  }
  return c;
}
static bool mb_txrx(const uint8_t* req, size_t rlen, uint8_t* rx, size_t& xlen, uint32_t to){
  while (XT.available()) XT.read();
  XT.write(req, rlen); XT.flush();
  uint32_t t0 = millis(); size_t i = 0;
  while ((millis() - t0) < to){
    while (XT.available()){
      if (i < xlen) rx[i++] = (uint8_t)XT.read();
    }
  }
  xlen = i;
  return i >= 5;
}
static bool mb_read_regs(uint8_t dev,uint8_t func,uint16_t reg,uint16_t cnt,uint8_t* data,uint8_t& nbytes){
  uint8_t req[8]={dev,func,(uint8_t)(reg>>8),(uint8_t)reg,(uint8_t)(cnt>>8),(uint8_t)cnt,0,0};
  uint16_t crc=mb_crc16(req,6); req[6]=crc&0xFF; req[7]=crc>>8;
  uint8_t rx[64]; size_t x=sizeof(rx);
  if(!mb_txrx(req,sizeof(req),rx,x,MB_RX_TIMEOUT)) return false;
  if(x<5) return false;
  uint16_t rxcrc=rx[x-2]|(rx[x-1]<<8);
  if(mb_crc16(rx,x-2)!=rxcrc) return false;
  if(rx[0]!=dev || rx[1]==(func|0x80) || rx[1]!=func) return false;
  uint8_t bc=rx[2]; if(bc!=cnt*2) return false;
  memcpy(data,rx+3,bc); nbytes=bc; return true;
}
static bool mb_write_u16(uint8_t dev,uint16_t reg,uint16_t val){
  uint8_t req[8]={dev,0x06,(uint8_t)(reg>>8),(uint8_t)reg,(uint8_t)(val>>8),(uint8_t)val,0,0};
  uint16_t crc=mb_crc16(req,6); req[6]=crc&0xFF; req[7]=crc>>8;
  uint8_t rx[16]; size_t x=sizeof(rx);
  if(!mb_txrx(req,sizeof(req),rx,x,MB_RX_TIMEOUT)) return false;
  if(x<8) return false;
  uint16_t rxcrc=rx[x-2]|(rx[x-1]<<8);
  return (mb_crc16(rx,x-2)==rxcrc && rx[0]==dev && rx[1]==0x06);
}



// ===== Helpers =====
static inline uint16_t clampU16(int v){ return (v<0)?0: (v>65535?65535:(uint16_t)v); }

// ===== BLE callbacks =====
class SvrCB : public NimBLEServerCallbacks {
 public:
  void onConnect(NimBLEServer* s) { (void)s; bleClientConnected = true; }
  void onConnect(NimBLEServer* s, ble_gap_conn_desc* /*desc*/) { onConnect(s); }
  void onDisconnect(NimBLEServer* s) { bleClientConnected = false; if (s) s->startAdvertising(); }
};

// Robust control write with Serial logging + ACK on STATUS
class CtrlCB : public NimBLECharacteristicCallbacks {
public:
  void onWrite(NimBLECharacteristic* c) { handleWrite(c); }
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& /*ci*/) { handleWrite(c); }
private:
  void handleWrite(NimBLECharacteristic* c) {
    if (!c) return;
    std::string v = c->getValue();
    Serial.printf("[BLE] CONTROL write (%u bytes): %s\n", (unsigned)v.size(), v.c_str());

    bool shouldArm = !v.empty();
    if (shouldArm && v.find("\"arm\"") != std::string::npos) {
      if (v.find(":1") == std::string::npos && v.find("true") == std::string::npos) {
        shouldArm = false;
      }
    }
    if (shouldArm) {
      armRequested = true;
      if (chStatus) {
        char js[160];
        const uint32_t now = millis();
        snprintf(js, sizeof(js), "{\"ack\":\"arm\",\"t_ms\":%lu,\"phase\":%u}",
                 (unsigned long)now, (unsigned)phase);
        chStatus->setValue((uint8_t*)js, strlen(js));
        if (bleClientConnected) chStatus->notify();
      }
    }
  }
};

// ===== OTA helpers =====
static void startOTAOnce() {
  if (gOTA) return;
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();
  gOTA = true;
}

static void startWiFiForOTA() {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) delay(100);
  if (WiFi.status() != WL_CONNECTED) {
    String ssid = String("LidarBeacon-") + String((uint32_t)ESP.getEfuseMac(), HEX).substring(6);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid.c_str(), "BeaconSetup");
  }
  startOTAOnce();
}

// ===== BLE start =====
static void startBLE() {
  NimBLEDevice::init("Lidar_Beacon");
  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new SvrCB());

  bleService = bleServer->createService(UUID_SERVICE);

  chStatus  = bleService->createCharacteristic(UUID_CHAR_STATUS,  NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  chControl = bleService->createCharacteristic(UUID_CHAR_CONTROL, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  chRange   = bleService->createCharacteristic(UUID_CHAR_RANGE,   NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  chSprint  = bleService->createCharacteristic(UUID_CHAR_SPRINT,  NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  chControl->setCallbacks(new CtrlCB());

  chStatus->setValue("{\"boot\":true}");
  uint16_t seed=0xFFFF; chRange->setValue((uint8_t*)&seed, 2);
  uint32_t zero=0;       chSprint->setValue((uint8_t*)&zero, 4);

  bleService->start();
  bleAdv = NimBLEDevice::getAdvertising();
  bleAdv->addServiceUUID(UUID_SERVICE);
  bleAdv->start();
}

// ===== TF-Luna parser =====
bool readLunaFrame(uint16_t& mmOut) {
  while (LUNA.available() >= 9) {
    if (LUNA.peek() != 0x59) { LUNA.read(); continue; }
    LUNA.read();
    if (LUNA.peek() != 0x59) continue;
    LUNA.read();
    uint8_t buf[7];
    if (LUNA.readBytes(buf, 7) != 7) return false;
    uint16_t dist = buf[0] | (uint16_t(buf[1]) << 8);
    mmOut = dist;
    return true;
  }
  return false;
}

/* ===== XT-S1 — 1 Hz ASCII poll with critical-phase block =====
   CHANGE: We only parse XT once per second, and we do nothing during COUNTDOWN/RUNNING.
*/
static const uint32_t XT_POLL_INTERVAL_MS = 250;   // 1 Hz
// Poll XT-S1 via Modbus; updates global xt_cm (int32_t)
// Pauses during COUNTDOWN/RUNNING (your critical phases)
static void pollXT_modbus(){
  if (phase == Phase::COUNTDOWN || phase == Phase::RUNNING) return; // never during critical timing
  static uint32_t lastPoll=0;
  uint32_t now=millis();
  if (now - lastPoll < MB_POLL_MS) return;
  lastPoll = now;

  bool updated=false;
  { // try 0x0017 (raw mm)
    uint8_t d[6]; uint8_t nb=sizeof(d);
    if (mb_read_regs(MB_DEV_ID, 0x04, 0x0017, 3, d, nb)){
      uint16_t raw_mm = (d[0]<<8)|d[1];
      if (raw_mm < 0xFFF0){ xt_cm = (int32_t)((raw_mm + 5)/10); updated=true; }
    }
  }
  if (!updated){ // fallback 0x002E (cm)
    uint8_t c[2]; uint8_t nb=sizeof(c);
    if (mb_read_regs(MB_DEV_ID, 0x04, 0x002E, 1, c, nb)){
      uint16_t raw_cm = (c[0]<<8)|c[1];
      if (raw_cm < 0xFFF0) xt_cm = (int32_t)raw_cm;
    }
  }
}


// ===== LED helpers =====
inline uint16_t XY(uint8_t x, uint8_t y) { return (uint16_t)y*8u + x; }

// Standby: ONLY one rotating blue row (everything else OFF)
void ledsStandby(uint32_t now) {
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  uint8_t row = (now / 250) % 8;   // step every 250ms
  const CRGB blue = CRGB(20, 100, 255);
  for (uint8_t x=0; x<8; ++x) leds[XY(x, row)] = blue;
}

void ledsFire(uint32_t /*now*/) {
  static byte heat[8];
  for (int i=0;i<8;i++) heat[i] = qsub8(heat[i], random8(0, 30));
  if (random8() < 140) heat[0] = qadd8(heat[0], random8(120,200));
  for (int k=7; k>=1; k--) heat[k] = (heat[k-1] + heat[k]) / 2;
  for (uint8_t y=0;y<8;y++) {
    CRGB c = HeatColor(heat[y]);
    for (uint8_t x=0;x<8;x++) leds[XY(x,7-y)] = c;
  }
}

void ledsGreenWaves(uint32_t now) {
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  for (uint8_t y=0;y<8;y++) {
    for (uint8_t x=0;x<8;x++) {
      uint8_t v = (sin8( (x*32) + (now/3) ) + sin8( (y*48) + (now/4) )) / 2;
      leds[XY(x,y)] = CRGB(0, v, 0);
    }
  }
}

void ledsRainbow(uint32_t now) {
  for (uint8_t y=0;y<8;y++) {
    for (uint8_t x=0;x<8;x++) {
      leds[XY(x,y)] = CHSV((x*16 + y*8 + (now/10)) & 0xFF, 255, 255);
    }
  }
}

// OTA breathing pulse (purple)
static void ledsOTAPulse(uint32_t now) {
  uint8_t v = beatsin8(12, 15, 180);     // slow, soft
  CRGB purple = CRGB(v/2, 0, v);
  fill_solid(leds, NUM_LEDS, purple);
}

// ===== Phase control =====
void enterPhase(Phase p) {
  phase = p;
  tPhaseStart = millis();
}

void armWithRandomCountdown() {
  enterPhase(Phase::ARMED);
  uint32_t seed = esp_timer_get_time() ^ millis() ^ (uint32_t)ESP.getEfuseMac();
  random16_add_entropy(seed & 0xFFFF);
  uint32_t dur = random(COUNTDOWN_MIN_MS, COUNTDOWN_MAX_MS + 1);
  countdownTargetMs = millis() + dur;
}

// ===== Setup =====
void setup() {
  // Serial debug
  Serial.begin(115200);
  uint32_t tWait = millis();
  while (!Serial && millis() - tWait < 1500) { delay(10); }
  Serial.println("\n[LIDAR_BEACON] Booting...");
  Serial.printf("TF-Luna UART2 RX=%d TX=%d @%d\n", LUNA_RX, LUNA_TX, LIDAR_BAUD);
  Serial.printf("XT-S1  UART1 RX=%d TX=%d @%d\n", XT_RX, XT_TX, LIDAR_BAUD);

  // FastLED init
  FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, NUM_LEDS);
  FastLED.setDither(false);
  FastLED.setBrightness(200);
  FastLED.clear(true);
  FastLED.show();

  // Button
  pinMode(PIN_BTN, INPUT_PULLUP);

  // Detect OTA boot-hold (LOW ≥ 250 ms)
  bool bootOTA = false;
  uint32_t t0 = millis();
  while (millis() - t0 < 250) {
    if (digitalRead(PIN_BTN) == HIGH) { bootOTA = false; break; }
    bootOTA = true;
  }
  Serial.printf("Boot OTA hold: %s\n", bootOTA ? "YES" : "NO");

  // Serial ports for LiDARs (safe even if not connected)
  LUNA.begin(LIDAR_BAUD, SERIAL_8N1, LUNA_RX, LUNA_TX);
  XT.begin(LIDAR_BAUD,   SERIAL_8N1, XT_RX,   XT_TX);
// XT config (best-effort; matches your working sketch)
mb_write_u16(MB_DEV_ID, 0x003D, 0x0001);  // enable output
mb_write_u16(MB_DEV_ID, 0x0042, 0x000A);  // set frame rate = 10 fps  ✅ (was 0x0032 = 50 fps)
mb_write_u16(MB_DEV_ID, 0x0003, 0x0001);  // start measurement

  // BLE always up (except we ignore it while in OTA loop)
  startBLE();

  if (bootOTA) {
    // Keep LEDs OFF until OTA is actually started
    FastLED.clear(true);
    FastLED.show();
    startWiFiForOTA();  // this sets gOTA=true via ArduinoOTA.begin() inside startOTAOnce()
  }

  enterPhase(Phase::STANDBY);
}

// ===== Loop =====
void loop() {
  ArduinoOTA.handle();
  const uint32_t now = millis();

  // If in OTA, show the purple pulse and skip BLE/LiDAR/state while flashing.
  if (gOTA) {
    ledsOTAPulse(now);
    FastLED.show();
    delay(10);
    return;  // BLE/ARM ignored while in OTA (per your request)
  }

  // ----- Button short/long -----
  const bool pressed = (digitalRead(PIN_BTN) == LOW);
  static bool lastPressed = false;
  if (pressed && !lastPressed) btnPressAt = now;
  if (!pressed && lastPressed) {
    const uint32_t dur = now - btnPressAt;
    if (dur < 1000) {
      Serial.println("[BTN] Short press → restart");
      ESP.restart();
    } else {
      Serial.println("[BTN] Long press → enter OTA");
      startWiFiForOTA();   // enters OTA; loop will early-return above
    }
  }
  lastPressed = pressed;

  // ----- LiDAR reads (non-blocking, safe if nothing attached) -----
  uint16_t mm;
  if (readLunaFrame(mm)) luna_mm = (int32_t)mm;

  // *** CHANGE: XT 1 Hz polling, blocked during COUNTDOWN/RUNNING
// XT-01 via Modbus (blocked during COUNTDOWN/RUNNING)
pollXT_modbus();


  // ----- Handle BLE arm command -----
  if (armRequested) {
    armRequested = false;
    if (phase == Phase::STANDBY || phase == Phase::FINISHED) {
      Serial.println("[ARM] Received – starting randomized countdown");
      armWithRandomCountdown();
    }
  }

  // ----- Phase machine -----
  switch (phase) {
    case Phase::STANDBY:
      ledsStandby(now);
      break;

    case Phase::ARMED:
      ledsFire(now);
      if ((int32_t)(now - countdownTargetMs) >= 0)
        enterPhase(Phase::COUNTDOWN);
      break;

    case Phase::COUNTDOWN:
      ledsFire(now);
      if ((int32_t)(now - countdownTargetMs) >= 0) {
        runStartMs = now;

        // (Re)initialize finish detection
        belowConsec = 0;
        aboveConsec = ABOVE_CONSEC_REQUIRED;
        nearArmed   = true;
        runIgnoreUntilMs = now + 300;   // ignore first 300 ms after start
        baseline_cm = BASELINE_MAX_CM;  // default if no valid reading
        if (luna_mm > 0) {
          uint16_t cm = (uint16_t)luna_mm;  // mm → cm (rounded)
          baseline_cm = (cm > BASELINE_MAX_CM) ? BASELINE_MAX_CM : cm;
        }
        Serial.println("[RUN] Started");
        enterPhase(Phase::RUNNING);
      }
      break;

    case Phase::RUNNING: {
      ledsGreenWaves(now);

      // Robust finish detection on "near" crossing
      if (luna_mm > 0 && (int32_t)(now - runIgnoreUntilMs) >= 0) {
        uint16_t cur_cm   = (uint16_t)luna_mm;
        // Can't be equal to zero
         if (luna_mm > 0) {
          cur_cm = (uint16_t)luna_mm; 
          if (cur_cm > BASELINE_MAX_CM) cur_cm = BASELINE_MAX_CM;
            }
  const uint16_t drop_cm  = (baseline_cm > cur_cm) ? (baseline_cm - cur_cm) : 0;



        if (cur_cm >= MIN_ABS_CM && drop_cm >= MIN_DROP_CM) {
            // FINISH!
            sprintMs = now - runStartMs;
            Serial.printf("[RUN] FINISH. sprint_ms=%lu\n", (unsigned long)sprintMs);

            if (chSprint) {
              chSprint->setValue((uint8_t*)&sprintMs, 4);
              if (bleClientConnected) chSprint->notify();
            }
            if (chStatus) {
              char js[192];
              snprintf(js, sizeof(js),
                "{\"finish\":true,\"sprint_ms\":%lu,\"phase\":%u,\"t_ms\":%lu}",
                (unsigned long)sprintMs, (unsigned)Phase::FINISHED, (unsigned long)now);
              chStatus->setValue((uint8_t*)js, strlen(js));
              if (bleClientConnected) chStatus->notify();
            }

            enterPhase(Phase::FINISHED);
          
        } else {
          if (belowConsec > 0) belowConsec--;
        }
      }
    } break;

    case Phase::FINISHED:
      ledsRainbow(now);
      if (now - tPhaseStart >= 5000)
        enterPhase(Phase::STANDBY);
      break;
  }

// ----- STATUS / RANGE updates -----
static uint32_t lastBle = 0;
if (now - lastBle >= 300) {
  lastBle = now;
  const bool have_start = (phase == Phase::RUNNING);
  const bool finished_now = (phase == Phase::FINISHED);

  // XT-only: lidar_cm strictly from XT-01 (ASCII already in cm)
  uint16_t ucm = 0xFFFF;
  if (xt_cm >= 0 && xt_cm < 65535) ucm = (uint16_t)xt_cm;

  char js[256];
  snprintf(js, sizeof(js),
    "{\"phase\":%u,\"have_start\":%s,\"finish\":%s,"
    "\"luna_mm\":%ld,\"xt_cm\":%ld,\"lidar_cm\":%u,"
    "\"uptime\":%lu,\"sprint_ms\":%lu}",
    (unsigned)phase,
    have_start ? "true" : "false",
    finished_now ? "true" : "false",
    (long)luna_mm, (long)xt_cm, (unsigned)ucm,
    (unsigned long)now, (unsigned long)sprintMs
  );

  if (chStatus) {
    chStatus->setValue((uint8_t*)js, strlen(js));
    if (bleClientConnected) chStatus->notify();
  }

  if (chRange) {
    chRange->setValue((uint8_t*)&ucm, 2);
    if (bleClientConnected) chRange->notify();
  }
}


  // ----- DEBUG ticker: print LiDARs & state every 200ms -----
  static uint32_t lastDbg = 0;
  if (now - lastDbg >= 200) {
    lastDbg = now;
    Serial.printf("[DBG] t=%lu phase=%u  luna_mm=%ld  xt_cm=%ld  below=%u  above=%u  armed=%d\n",
                  (unsigned long)now, (unsigned)phase,
                  (long)luna_mm, (long)xt_cm,
                  (unsigned)belowConsec, (unsigned)aboveConsec, (int)nearArmed);
  }

  FastLED.show();
  delay(0);
}
