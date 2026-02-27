// =====================================================
// Arduino Nano I/O bridge for dual-pump project
// Nano keeps original wiring (sensors/relay/RS485 VFD),
// while all control logic is moved to ESP32.
// =====================================================

#include <SoftwareSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <avr/wdt.h>

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

Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);

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

  // Display runtime
  unsigned long lastDisplayUpdate = 0;
  unsigned long flash = 0;
  bool flashOn = false;
} ns;

const unsigned long DISPLAY_UPDATE_MS = 1000;
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

void drawStatic() {
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(5, 10);   tft.print("S:");
  tft.setCursor(70, 10);  tft.print("A:");
  tft.setCursor(135, 10); tft.print("W:");

  tft.setTextSize(1);
  for (uint8_t i = 0; i < 4; i++) {
    int x = 10 + i * 55;
    tft.drawRoundRect(x, Y2 + 15, 30, 30, 5, GRAY);
    tft.setCursor(x + 11, Y2 + 25); tft.print(i + 1);
  }

  tft.setCursor(5, Y3 + 10);   tft.print("I1:");
  tft.setCursor(5, Y3 + 30);   tft.print("T1:");
  tft.setCursor(5, Y3 + 50);   tft.print("T2:");
  tft.setCursor(120, Y3 + 10); tft.print("V1:");
  tft.setCursor(120, Y3 + 30); tft.print("P1:");

  tft.setCursor(5, Y4 + 10);   tft.print("I2:");
  tft.setCursor(5, Y4 + 30);   tft.print("H2:");
  tft.setCursor(5, Y4 + 50);   tft.print("P2:");
  tft.setCursor(120, Y4 + 10); tft.print("ST2:");

  tft.setCursor(5, Y5 + 5);    tft.print("FIL:");
  tft.setCursor(120, Y5 + 5);  tft.print("PRO:");
}

void updateDisplay(unsigned long now) {
  bool espStatus = useEspDisplayStatus(now);
  bool wellAlarm = espStatus ? ns.wellAlarm : false;
  bool wellBlocked = espStatus ? ns.wellBlocked : false;
  bool houseAlarm = espStatus ? ns.houseAlarm : false;
  bool houseBlocked = espStatus ? ns.houseBlocked : false;
  bool filterWarning = ns.wellPressureBar >= WELL_PRESSURE_WARNING;
  bool pressureBlock = ns.wellPressureBar >= WELL_PRESSURE_BLOCK;

  tft.fillRect(25, 10, 35, 16, BLACK); tft.setTextSize(2); tft.setTextColor(GREEN); tft.setCursor(25, 10); tft.print("ON");
  tft.fillRect(90, 10, 35, 16, BLACK); tft.setTextColor((wellAlarm || houseAlarm) ? RED : GREEN); tft.setCursor(90, 10); tft.print((wellAlarm || houseAlarm) ? "Y" : "N");
  tft.fillRect(155, 10, 35, 16, BLACK); tft.setTextColor(filterWarning ? YELLOW : GREEN); tft.setCursor(155, 10); tft.print(filterWarning ? "Y" : "N");

  for (uint8_t i = 0; i < 4; i++) {
    int x = 10 + i * 55;
    tft.fillRoundRect(x, Y2 + 15, 30, 30, 5, ns.levels[i] ? GREEN : GRAY);
    tft.setTextColor(BLACK); tft.setCursor(x + 11, Y2 + 25); tft.print(i + 1);
  }

  tft.setTextSize(2);
  tft.fillRect(25, Y3 + 10, 90, 16, BLACK);
  tft.setTextColor(ns.wellCurrent > WELL_CURRENT_OVERLOAD ? RED : ns.wellCurrent < WELL_CURRENT_DRY ? YELLOW : WHITE);
  tft.setCursor(25, Y3 + 10); tft.print(ns.wellCurrent, 1); tft.print("A");

  tft.fillRect(25, Y3 + 30, 90, 16, BLACK);
  tft.setTextColor(WHITE);
  tft.setCursor(25, Y3 + 30); tft.print(espStatus ? wellModeText(ns.wellMode) : (ns.relayWellOn ? "RUN" : "WAIT"));

  tft.fillRect(25, Y3 + 50, 90, 16, BLACK);
  tft.setTextColor(WHITE);
  tft.setCursor(25, Y3 + 50); tft.print(espStatus ? intentionText(ns.wellIntention) : "LOCAL");

  tft.fillRect(140, Y3 + 10, 80, 16, BLACK);
  tft.setTextColor(WHITE);
  tft.setCursor(140, Y3 + 10); tft.print(ns.vfdRun ? "RUN" : "STOP");

  tft.fillRect(140, Y3 + 30, 80, 16, BLACK);
  tft.setTextColor(filterWarning ? YELLOW : pressureBlock ? RED : WHITE);
  tft.setCursor(140, Y3 + 30); tft.print(ns.wellPressureBar, 2); tft.print("b");

  tft.setTextSize(2);
  tft.fillRect(25, Y4 + 10, 70, 16, BLACK);
  tft.setCursor(25, Y4 + 10);
  if (ns.houseCurrent >= HOUSE_CURRENT_EMERGENCY) {
    tft.setTextColor(RED);
  } else if (ns.houseCurrent < HOUSE_CURRENT_DRY) {
    tft.setTextColor(YELLOW);
  } else if (ns.houseCurrent > HOUSE_CURRENT_OVERLOAD) {
    tft.setTextColor(RED);
  } else {
    tft.setTextColor(WHITE);
  }
  tft.print(ns.houseCurrent, 1); tft.print("A");

  tft.fillRect(25, Y4 + 30, 70, 16, BLACK);
  tft.setCursor(25, Y4 + 30);
  tft.setTextColor(WHITE);
  tft.print(ns.vfdFreqHz, 1); tft.print("H");

  tft.fillRect(25, Y4 + 50, 70, 16, BLACK);
  tft.setCursor(25, Y4 + 50);
  tft.setTextColor(WHITE);
  tft.print(ns.housePressureBar, 1); tft.print("b");

  tft.fillRect(140, Y4 + 10, 70, 16, BLACK);
  tft.setCursor(140, Y4 + 10);
  if (houseBlocked) {
    tft.setTextColor(RED);
    tft.print("BLOCK");
  } else {
    tft.setTextColor(ns.vfdRun ? GREEN : YELLOW);
    tft.print(espStatus ? houseModeText(ns.houseMode) : (ns.vfdRun ? "RUN" : "WAIT"));
  }

  tft.setTextSize(1);
  tft.fillRect(30, Y5 + 5, 60, 10, BLACK);
  tft.setTextColor(filterWarning ? YELLOW : GREEN);
  tft.setCursor(30, Y5 + 5); tft.print(filterWarning ? "BAD" : "OK");

  tft.fillRect(150, Y5 + 5, 60, 10, BLACK);
  bool protectionOff = wellBlocked || houseBlocked || pressureBlock;
  tft.setTextColor(protectionOff ? RED : GREEN);
  tft.setCursor(150, Y5 + 5); tft.print(protectionOff ? "OFF" : "ON");
}

void handleFlashing(unsigned long now) {
  bool espStatus = useEspDisplayStatus(now);
  bool wellAlarm = espStatus ? ns.wellAlarm : false;
  bool houseAlarm = espStatus ? ns.houseAlarm : false;
  bool filterWarning = ns.wellPressureBar >= WELL_PRESSURE_WARNING;
  bool pressureBlock = ns.wellPressureBar >= WELL_PRESSURE_BLOCK;

  if (!wellAlarm && !houseAlarm && !filterWarning && !pressureBlock) return;
  if (now - ns.flash < 500) return;

  ns.flash = now;
  ns.flashOn = !ns.flashOn;

  uint16_t color = pressureBlock ? RED : (filterWarning || houseAlarm) ? YELLOW : RED;
  tft.fillRect(0, 0, 240, Z1, ns.flashOn ? color : BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ns.flashOn ? BLACK : WHITE);
  tft.setCursor(5, 10); tft.print("S:");
  tft.setCursor(70, 10); tft.print("A:");
  tft.setCursor(135, 10); tft.print("W:");
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

void handleCommand(String cmd) {
  cmd.trim();
  int pos = 0;
  while (pos < cmd.length()) {
    int sep = cmd.indexOf(';', pos);
    if (sep < 0) sep = cmd.length();

    String token = cmd.substring(pos, sep);
    int eq = token.indexOf('=');
    if (eq > 0) {
      String key = token.substring(0, eq);
      String val = token.substring(eq + 1);
      if (key == "RELAY") ns.relayWellOn = val.toInt() == 1;
      if (key == "VFD_RUN") ns.vfdRun = val.toInt() == 1;
      if (key == "VFD_FREQ") ns.vfdFreqHz = val.toFloat();
      if (key == "WELL_MODE") { ns.wellMode = val.toInt(); ns.statusFromEsp = true; ns.statusUpdatedAt = millis(); }
      if (key == "WELL_ALARM") { ns.wellAlarm = val.toInt() == 1; ns.statusFromEsp = true; ns.statusUpdatedAt = millis(); }
      if (key == "WELL_BLOCKED") { ns.wellBlocked = val.toInt() == 1; ns.statusFromEsp = true; ns.statusUpdatedAt = millis(); }
      if (key == "WELL_INTENTION") { ns.wellIntention = val.toInt(); ns.statusFromEsp = true; ns.statusUpdatedAt = millis(); }
      if (key == "HOUSE_MODE") { ns.houseMode = val.toInt(); ns.statusFromEsp = true; ns.statusUpdatedAt = millis(); }
      if (key == "HOUSE_ALARM") { ns.houseAlarm = val.toInt() == 1; ns.statusFromEsp = true; ns.statusUpdatedAt = millis(); }
      if (key == "HOUSE_BLOCKED") { ns.houseBlocked = val.toInt() == 1; ns.statusFromEsp = true; ns.statusUpdatedAt = millis(); }
    }
    pos = sep + 1;
  }
}

void processEspUart() {
  static String line;
  while (espSerial.available()) {
    char c = (char)espSerial.read();
    if (c == '\n') {
      handleCommand(line);
      line = "";
    } else if (c != '\r') {
      line += c;
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

  tft.init(240, 320);
  tft.setRotation(2);
  tft.fillScreen(BLACK);
  drawStatic();

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

  if (now - ns.lastDisplayUpdate >= DISPLAY_UPDATE_MS) {
    ns.lastDisplayUpdate = now;
    updateDisplay(now);
    feedWatchdog();
  }
  handleFlashing(now);
  feedWatchdog();
}
