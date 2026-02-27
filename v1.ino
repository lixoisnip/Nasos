// =====================================================
// Arduino Nano I/O bridge for dual-pump project
// Nano keeps original wiring (sensors/relay/RS485 VFD),
// while all control logic is moved to ESP32.
// =====================================================

#include <SoftwareSerial.h>
#include <avr/wdt.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

// Set to 1 to enable TFT diagnostics on Nano.
// Default is 0 to keep firmware size below ATmega328P limit.
#ifndef NANO_USE_TFT
#define NANO_USE_TFT 0
#endif

#if NANO_USE_TFT
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#endif

// Original project pins (unchanged wiring)
#define RELAY_WELL        3
#define PIN_RS485_DE_RE   7
#define ACS_PIN           A1
#define PIN_CURRENT       A0
#define PRESSURE_PIN      A2
#define PIN_PRESSURE      A3
#define L1                A4
#define L2                A5
#define L3                A6
#define L4                A7

// TFT pins (same as Osnova.ino)
#define TFT_CS           10
#define TFT_DC            9
#define TFT_RST           8

#if NANO_USE_TFT
Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);
#endif

#define BLACK   ST77XX_BLACK
#define WHITE   ST77XX_WHITE
#define GREEN   ST77XX_GREEN
#define RED     ST77XX_RED
#define YELLOW  ST77XX_YELLOW
#define GRAY    0x7BEF

#define Z1 40
#define Z2 60
#define Z3 80
#define Z4 80
#define Z5 20
#define Y1 0
#define Y2 (Y1+Z1)
#define Y3 (Y2+Z2)
#define Y4 (Y3+Z3)
#define Y5 (Y4+Z4)

// UART to ESP32 (SoftwareSerial to avoid conflict with RS485 on Serial)
#define ESP_RX_PIN        4
#define ESP_TX_PIN        5
SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);

const uint8_t WELL_CURRENT_SAMPLES = 120;
const uint8_t HOUSE_PRESSURE_AVG_SAMPLES = 6;
const unsigned long LEVEL_FILTER_MS = 2000;
const int LEVEL_THRESH = 700;
const size_t ESP_CMD_MAX_LEN = 160;

struct LevelFilter {
  bool stableState = false;
  bool pendingState = false;
  unsigned long pendingSince = 0;
};

struct NanoState {
  float wellCurrent = 0.0f;
  float houseCurrent = 0.0f;
  float wellPressureBar = 0.0f;
  float housePressureBar = 0.0f;
  bool levels[4] = {false, false, false, false};

  // Actuator targets from ESP32
  bool relayWellOn = false;
  bool vfdRun = false;
  float vfdFreqHz = 0.0f;

  // Runtime
  float currentZeroOffset = 512.0f;
  unsigned long lastTelemetry = 0;
  LevelFilter levelFilters[4];
  float housePressureHistory[HOUSE_PRESSURE_AVG_SAMPLES] = {0};
  uint8_t housePressureIndex = 0;
  uint8_t housePressureCount = 0;

  // ESP32 display status (optional source)
  int wellMode = -1;
  bool wellAlarm = false;
  bool wellBlocked = false;
  int wellIntention = -1;
  int houseMode = -1;
  bool houseAlarm = false;
  bool houseBlocked = false;
  bool statusFromEsp = false;
  unsigned long statusUpdatedAt = 0;
  unsigned long uartRxErrorCount = 0;

} ns;

const unsigned long ESP_STATUS_TIMEOUT_MS = 5000;

const float WELL_CURRENT_DRY = 3.3f;
const float WELL_CURRENT_OVERLOAD = 4.3f;
const float WELL_PRESSURE_WARNING = 1.2f;
const float WELL_PRESSURE_BLOCK = 1.5f;

const float HOUSE_CURRENT_DRY = 0.4f;
const float HOUSE_CURRENT_OVERLOAD = 1.3f;
const float HOUSE_CURRENT_EMERGENCY = 1.5f;

bool useEspDisplayStatus(unsigned long now) {
  return ns.statusFromEsp && (now - ns.statusUpdatedAt <= ESP_STATUS_TIMEOUT_MS);
}

const char* wellModeText(int mode) {
  switch (mode) {
    case 0: return "WAIT";
    case 1: return "START";
    case 2: return "RUN";
    case 3: return "FAIL";
    default: return "WAIT";
  }
}

const char* houseModeText(int mode) {
  switch (mode) {
    case 0: return "WAIT";
    case 1: return "READY";
    case 2: return "RUN";
    case 3: return "STOP";
    default: return ns.vfdRun ? "RUN" : "WAIT";
  }
}

const char* intentionText(int intention) {
  switch (intention) {
    case 1: return "TARGET";
    case 2: return "PUMP_L4";
    default: return "UNKNOWN";
  }
}

void feedWatchdog() {
  wdt_reset();
}

uint8_t calcXorChecksum(const String &payload) {
  uint8_t checksum = 0;
  for (int i = 0; i < payload.length(); i++) checksum ^= (uint8_t)payload[i];
  return checksum;
}

void txMode() {
  digitalWrite(PIN_RS485_DE_RE, HIGH);
  delayMicroseconds(100);
}

void rxMode() {
  delayMicroseconds(100);
  digitalWrite(PIN_RS485_DE_RE, LOW);
}

uint16_t calculateCRC(uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return crc;
}

void writeReg(uint16_t addr, uint16_t val) {
  feedWatchdog();
  uint8_t frame[8];
  frame[0] = 0x08;
  frame[1] = 0x06;
  frame[2] = highByte(addr);
  frame[3] = lowByte(addr);
  frame[4] = highByte(val);
  frame[5] = lowByte(val);

  uint16_t crc = calculateCRC(frame, 6);
  frame[6] = lowByte(crc);
  frame[7] = highByte(crc);

  txMode();
  Serial.write(frame, 8);
  Serial.flush();
  rxMode();
  delay(20);
  feedWatchdog();
}

void vfdStart() { writeReg(0x9CA7, 0x0001); }
void vfdStop()  { writeReg(0x9CA7, 0x0000); }

void setFrequency(float hz) {
  uint16_t freq = (uint16_t)round(max(0.0f, hz) * 100.0f);
  writeReg(0x9CA6, freq);
}

void initVFD() {
  feedWatchdog();
  writeReg(0x9C41, 0x0002); delay(150); feedWatchdog();
  writeReg(0x9C40, 0x0005); delay(150); feedWatchdog();
  writeReg(0x9CA6, 0x0000); delay(150); feedWatchdog();
  vfdStop();
}

float readWellCurrent() {
  float sqSum = 0.0f;
  for (uint8_t i = 0; i < WELL_CURRENT_SAMPLES; i++) {
    float delta = analogRead(ACS_PIN) - ns.currentZeroOffset;
    sqSum += delta * delta;
  }

  float rmsRaw = sqrt(sqSum / WELL_CURRENT_SAMPLES);
  float amps = rmsRaw * (5.0f / 1023.0f) / 0.066f;
  return amps < 0.10f ? 0.0f : amps;
}

float readHouseCurrent() {
  int raw = analogRead(PIN_CURRENT);
  float voltage = raw * 5.0f / 1023.0f;
  float amps = voltage * 2.0f;
  return amps < 0.10f ? 0.0f : amps;
}

float readWellPressureBar() {
  int raw = analogRead(PRESSURE_PIN);
  float voltage = raw * 5.0f / 1023.0f;
  return max(0.0f, (voltage - 0.5f) * 12.0f / 5.2f);
}

float readHousePressureBar() {
  int raw = analogRead(PIN_PRESSURE);
  float voltage = raw * 5.0f / 1023.0f;
  float rawBar = max(0.0f, (voltage - 0.5f) * 12.0f / 4.0f);

  float corrected = rawBar - 0.152f;
  corrected *= (1.80f / 1.95f);
  corrected = max(0.0f, corrected);

  ns.housePressureHistory[ns.housePressureIndex] = corrected;
  ns.housePressureIndex = (ns.housePressureIndex + 1) % HOUSE_PRESSURE_AVG_SAMPLES;
  if (ns.housePressureCount < HOUSE_PRESSURE_AVG_SAMPLES) ns.housePressureCount++;

  float sum = 0.0f;
  for (uint8_t i = 0; i < ns.housePressureCount; i++) sum += ns.housePressureHistory[i];
  return ns.housePressureCount ? (sum / ns.housePressureCount) : corrected;
}

bool readLevelFiltered(uint8_t idx, uint8_t pin, unsigned long now) {
  bool measuredState = analogRead(pin) > LEVEL_THRESH;
  LevelFilter &filter = ns.levelFilters[idx];

  if (measuredState != filter.stableState) {
    if (measuredState != filter.pendingState) {
      filter.pendingState = measuredState;
      filter.pendingSince = now;
    } else if (now - filter.pendingSince >= LEVEL_FILTER_MS) {
      filter.stableState = measuredState;
    }
  } else {
    filter.pendingState = filter.stableState;
    filter.pendingSince = now;
  }

  return filter.stableState;
}

void readInputs(unsigned long now) {
  ns.wellCurrent = readWellCurrent();
  ns.houseCurrent = readHouseCurrent();
  ns.wellPressureBar = readWellPressureBar();
  ns.housePressureBar = readHousePressureBar();

  ns.levels[0] = readLevelFiltered(0, L1, now);
  ns.levels[1] = readLevelFiltered(1, L2, now);
  ns.levels[2] = readLevelFiltered(2, L3, now);
  ns.levels[3] = readLevelFiltered(3, L4, now);
}

void applyOutputs() {
  // relay active LOW in original project
  digitalWrite(RELAY_WELL, ns.relayWellOn ? LOW : HIGH);

  static bool prevRun = false;
  static float prevFreq = -1;
  if (ns.vfdRun != prevRun) {
    ns.vfdRun ? vfdStart() : vfdStop();
    prevRun = ns.vfdRun;
  }

  if (abs(ns.vfdFreqHz - prevFreq) >= 0.1f) {
    setFrequency(ns.vfdFreqHz);
    prevFreq = ns.vfdFreqHz;
  }
}

void handleCommand(char* cmd) {
  char* token = strtok(cmd, ";");
  while (token != nullptr) {
    char* eq = strchr(token, '=');
    if (eq != nullptr && eq != token) {
      *eq = '\0';
      const char* key = token;
      const char* val = eq + 1;
      if (strcmp(key, "RELAY") == 0) ns.relayWellOn = atoi(val) == 1;
      if (strcmp(key, "VFD_RUN") == 0) ns.vfdRun = atoi(val) == 1;
      if (strcmp(key, "VFD_FREQ") == 0) ns.vfdFreqHz = atof(val);
      if (strcmp(key, "WELL_MODE") == 0) { ns.wellMode = atoi(val); ns.statusFromEsp = true; ns.statusUpdatedAt = millis(); }
      if (strcmp(key, "WELL_ALARM") == 0) { ns.wellAlarm = atoi(val) == 1; ns.statusFromEsp = true; ns.statusUpdatedAt = millis(); }
      if (strcmp(key, "WELL_BLOCKED") == 0) { ns.wellBlocked = atoi(val) == 1; ns.statusFromEsp = true; ns.statusUpdatedAt = millis(); }
      if (strcmp(key, "WELL_INTENTION") == 0) { ns.wellIntention = atoi(val); ns.statusFromEsp = true; ns.statusUpdatedAt = millis(); }
      if (strcmp(key, "HOUSE_MODE") == 0) { ns.houseMode = atoi(val); ns.statusFromEsp = true; ns.statusUpdatedAt = millis(); }
      if (strcmp(key, "HOUSE_ALARM") == 0) { ns.houseAlarm = atoi(val) == 1; ns.statusFromEsp = true; ns.statusUpdatedAt = millis(); }
      if (strcmp(key, "HOUSE_BLOCKED") == 0) { ns.houseBlocked = atoi(val) == 1; ns.statusFromEsp = true; ns.statusUpdatedAt = millis(); }
    }
    token = strtok(nullptr, ";");
  }
}

void processEspUart() {
  static char line[ESP_CMD_MAX_LEN + 1];
  static size_t idx = 0;
  static bool droppingFrame = false;
  while (espSerial.available()) {
    char c = (char)espSerial.read();

    if (c == '\n') {
      if (!droppingFrame && idx > 0) {
        line[idx] = '\0';
        handleCommand(line);
      }
      idx = 0;
      droppingFrame = false;
    } else if (c == '\r') {
      continue;
    } else if (!isprint((unsigned char)c)) {
      continue;
    } else if (droppingFrame) {
      continue;
    } else {
      if (idx >= ESP_CMD_MAX_LEN) {
        idx = 0;
        droppingFrame = true;
        ns.uartRxErrorCount++;
      } else {
        line[idx++] = c;
      }
    }
  }
}

void sendTelemetry(unsigned long now) {
  if (now - ns.lastTelemetry < 100) return;
  ns.lastTelemetry = now;

  String payload = "TEL,";
  payload += String(now);
  payload += ',' + String(ns.wellCurrent, 3);
  payload += ',' + String(ns.wellPressureBar, 3);
  payload += ',' + String(ns.houseCurrent, 3);
  payload += ',' + String(ns.housePressureBar, 3);
  payload += ',' + String(ns.levels[0] ? 1 : 0);
  payload += ',' + String(ns.levels[1] ? 1 : 0);
  payload += ',' + String(ns.levels[2] ? 1 : 0);
  payload += ',' + String(ns.levels[3] ? 1 : 0);
  payload += ',' + String(ns.vfdRun ? 1 : 0);
  payload += ',' + String(ns.vfdFreqHz, 1);

  uint8_t checksum = calcXorChecksum(payload);
  char frame[180];
  snprintf(frame, sizeof(frame), "%s*%02X", payload.c_str(), checksum);
  espSerial.println(frame);
}

void setup() {
  Serial.begin(9600);      // RS485 VFD
  espSerial.begin(38400);  // link to ESP32

  pinMode(RELAY_WELL, OUTPUT);
  digitalWrite(RELAY_WELL, HIGH);

  pinMode(PIN_RS485_DE_RE, OUTPUT);
  digitalWrite(PIN_RS485_DE_RE, LOW);

  long sum = 0;
  for (int i = 0; i < 600; i++) {
    sum += analogRead(ACS_PIN);
    if ((i % 50) == 0) feedWatchdog();
    delay(2);
  }
  ns.currentZeroOffset = sum / 600.0f;
  if (ns.currentZeroOffset < 400 || ns.currentZeroOffset > 600) ns.currentZeroOffset = 512.0f;

  initVFD();

  wdt_enable(WDTO_4S);
  feedWatchdog();
}

void loop() {
  feedWatchdog();
  unsigned long now = millis();
  processEspUart();
  feedWatchdog();
  readInputs(now);
  applyOutputs();
  feedWatchdog();
  sendTelemetry(now);

  feedWatchdog();
}
