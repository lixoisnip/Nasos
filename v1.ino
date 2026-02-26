// =====================================================
// Arduino Nano I/O bridge for dual-pump project
// Reads sensors, filters inputs, drives relay/VFD and sends telemetry to ESP32
// =====================================================

#include <SoftwareSerial.h>
#include <avr/wdt.h>
#include <math.h>

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

#define ESP_RX_PIN        4
#define ESP_TX_PIN        5
SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);

constexpr unsigned long TELEMETRY_PERIOD_MS = 150;
constexpr unsigned long LEVEL_FILTER_MS = 2000;
constexpr int LEVEL_THRESHOLD = 700;

struct LevelFilter {
  bool stable = false;
  bool pending = false;
  unsigned long changedAt = 0;
};

struct NanoState {
  float wellCurrent = 0.0f;
  float houseCurrent = 0.0f;
  float wellPressureBar = 0.0f;
  float housePressureBar = 0.0f;
  bool levels[4] = {false, false, false, false};

  bool relayWellOn = false;
  bool vfdRun = false;
  float vfdFreqHz = 0.0f;

  float currentZeroOffset = 512.0f;
  unsigned long lastTelemetry = 0;
  float housePressureBuf[6] = {0};
  uint8_t housePressureIdx = 0;
  bool housePressureInit = false;
  LevelFilter levelFilter[4];
} ns;

void txMode() { digitalWrite(PIN_RS485_DE_RE, HIGH); delayMicroseconds(100); }
void rxMode() { delayMicroseconds(100); digitalWrite(PIN_RS485_DE_RE, LOW); }

uint16_t calculateCRC(const uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i];
    for (uint8_t j = 0; j < 8; j++) crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

uint8_t xorChecksum(const String &s) {
  uint8_t c = 0;
  for (size_t i = 0; i < s.length(); i++) c ^= (uint8_t)s[i];
  return c;
}

void writeReg(uint16_t addr, uint16_t val) {
  uint8_t frame[8];
  frame[0] = 0x08; frame[1] = 0x06;
  frame[2] = highByte(addr); frame[3] = lowByte(addr);
  frame[4] = highByte(val);  frame[5] = lowByte(val);
  uint16_t crc = calculateCRC(frame, 6);
  frame[6] = lowByte(crc); frame[7] = highByte(crc);
  txMode();
  Serial.write(frame, 8);
  Serial.flush();
  rxMode();
  delay(20);
}

void vfdStart() { writeReg(0x9CA7, 0x0001); }
void vfdStop()  { writeReg(0x9CA7, 0x0000); }

void setFrequency(float hz) {
  uint16_t freq = (uint16_t)round(max(0.0f, hz) * 100.0f);
  writeReg(0x9CA6, freq);
}

void initVFD() {
  writeReg(0x9C41, 0x0002); delay(150);
  writeReg(0x9C40, 0x0005); delay(150);
  writeReg(0x9CA6, 0x0000); delay(150);
  vfdStop();
}

float readWellCurrent() {
  const int samples = 120;
  float sumSq = 0.0f;
  for (int i = 0; i < samples; i++) {
    float diff = analogRead(ACS_PIN) - ns.currentZeroOffset;
    sumSq += diff * diff;
  }
  float rms = sqrt(sumSq / samples);
  float voltage = rms * (5.0f / 1023.0f);
  float amps = voltage / 0.066f;
  return amps < 0.10f ? 0.0f : amps;
}

float readHouseCurrent() {
  int raw = analogRead(PIN_CURRENT);
  float voltage = raw * 5.0f / 1023.0f;
  float amps = voltage * 2.0f;
  return amps < 0.10f ? 0.0f : amps;
}

float readPressureBar(uint8_t pin) {
  float voltage = analogRead(pin) * 5.0f / 1023.0f;
  float bar = max(0.0f, (voltage - 0.5f) * 12.0f);
  if (pin == PRESSURE_PIN) {
    return max(0.0f, bar / 5.2f);
  }
  float corrected = bar - 0.152f;
  corrected *= (1.80f / 1.95f);
  return max(0.0f, corrected);
}

bool readLevelRaw(uint8_t pin) { return analogRead(pin) > LEVEL_THRESHOLD; }

void updateLevelFilter(uint8_t idx, bool raw, unsigned long now) {
  LevelFilter &f = ns.levelFilter[idx];
  if (raw != f.pending) {
    f.pending = raw;
    f.changedAt = now;
  }
  if (f.stable != f.pending && (now - f.changedAt) >= LEVEL_FILTER_MS) {
    f.stable = f.pending;
  }
  ns.levels[idx] = f.stable;
}

void readInputs(unsigned long now) {
  ns.wellCurrent = readWellCurrent();
  ns.houseCurrent = readHouseCurrent();
  ns.wellPressureBar = readPressureBar(PRESSURE_PIN);

  float hp = readPressureBar(PIN_PRESSURE);
  if (!ns.housePressureInit) {
    for (uint8_t i = 0; i < 6; i++) ns.housePressureBuf[i] = hp;
    ns.housePressureInit = true;
  }
  ns.housePressureBuf[ns.housePressureIdx] = hp;
  ns.housePressureIdx = (ns.housePressureIdx + 1) % 6;
  float sum = 0.0f;
  for (uint8_t i = 0; i < 6; i++) sum += ns.housePressureBuf[i];
  ns.housePressureBar = sum / 6.0f;

  updateLevelFilter(0, readLevelRaw(L1), now);
  updateLevelFilter(1, readLevelRaw(L2), now);
  updateLevelFilter(2, readLevelRaw(L3), now);
  updateLevelFilter(3, readLevelRaw(L4), now);
}

void applyOutputs() {
  digitalWrite(RELAY_WELL, ns.relayWellOn ? LOW : HIGH);
  static bool prevRun = false;
  static float prevFreq = -1;
  if (ns.vfdRun != prevRun) {
    ns.vfdRun ? vfdStart() : vfdStop();
    prevRun = ns.vfdRun;
  }
  if (fabs(ns.vfdFreqHz - prevFreq) >= 0.1f) {
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
  if (now - ns.lastTelemetry < TELEMETRY_PERIOD_MS) return;
  ns.lastTelemetry = now;

  String payload = "TEL," + String(now);
  payload += "," + String(ns.wellCurrent, 3);
  payload += "," + String(ns.wellPressureBar, 3);
  payload += "," + String(ns.houseCurrent, 3);
  payload += "," + String(ns.housePressureBar, 3);
  payload += "," + String(ns.levels[0] ? 1 : 0);
  payload += "," + String(ns.levels[1] ? 1 : 0);
  payload += "," + String(ns.levels[2] ? 1 : 0);
  payload += "," + String(ns.levels[3] ? 1 : 0);
  payload += "," + String(ns.vfdRun ? 1 : 0);
  payload += "," + String(ns.vfdFreqHz, 1);

  uint8_t csum = xorChecksum(payload);
  char hex[5];
  snprintf(hex, sizeof(hex), "%02X", csum);
  espSerial.print(payload);
  espSerial.print(",*");
  espSerial.println(hex);
}

void setup() {
  wdt_enable(WDTO_2S);
  Serial.begin(9600);
  espSerial.begin(38400);
  pinMode(RELAY_WELL, OUTPUT);
  digitalWrite(RELAY_WELL, HIGH);
  pinMode(PIN_RS485_DE_RE, OUTPUT);
  digitalWrite(PIN_RS485_DE_RE, LOW);

  long sum = 0;
  for (int i = 0; i < 600; i++) {
    sum += analogRead(ACS_PIN);
    delay(2);
  }
  ns.currentZeroOffset = sum / 600.0f;
  if (ns.currentZeroOffset < 400 || ns.currentZeroOffset > 600) ns.currentZeroOffset = 512.0f;

  unsigned long now = millis();
  for (uint8_t i = 0; i < 4; i++) {
    bool lvl = readLevelRaw(i == 0 ? L1 : (i == 1 ? L2 : (i == 2 ? L3 : L4)));
    ns.levelFilter[i].stable = lvl;
    ns.levelFilter[i].pending = lvl;
    ns.levelFilter[i].changedAt = now;
    ns.levels[i] = lvl;
  }

  initVFD();
}

void loop() {
  wdt_reset();
  unsigned long now = millis();
  processEspUart();
  readInputs(now);
  applyOutputs();
  sendTelemetry(now);
}
