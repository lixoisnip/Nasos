// =====================================================
// Arduino Nano I/O bridge for dual-pump project
// Nano keeps original wiring (sensors/relay/RS485 VFD),
// while all control logic is moved to ESP32.
// =====================================================

#include <Wire.h>
#include <avr/wdt.h>
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
#define L1                4
#define L2                5
#define L3                A6
#define L4                A7

// TFT pins (same as Osnova.ino)
#define TFT_CS           10
#define TFT_DC            9
#define TFT_RST           8

#if NANO_USE_TFT
Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);
#endif

#if NANO_USE_TFT
#define BLACK   ST77XX_BLACK
#define WHITE   ST77XX_WHITE
#define GREEN   ST77XX_GREEN
#define RED     ST77XX_RED
#define YELLOW  ST77XX_YELLOW
#define GRAY    0x7BEF
#endif

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

// I2C link to ESP32 (Nano as slave)
constexpr uint8_t NANO_I2C_ADDRESS = 0x2A;
namespace nanoProto {
constexpr uint8_t MAGIC = 0xA5;
constexpr uint8_t VERSION = 2;
constexpr uint8_t MSG_COMMAND = 1;
constexpr uint8_t MSG_TELEMETRY = 2;
constexpr uint8_t FLAG_WELL_ALARM = 1 << 0;
constexpr uint8_t FLAG_WELL_BLOCKED = 1 << 1;
constexpr uint8_t FLAG_HOUSE_ALARM = 1 << 2;
constexpr uint8_t FLAG_HOUSE_BLOCKED = 1 << 3;
constexpr uint8_t STATUS_VFD_RUN = 1 << 0;
constexpr uint8_t STATUS_CMD_STALE = 1 << 1;
constexpr uint8_t STATUS_LINK_OK = 1 << 2;
constexpr uint8_t COMMAND_FRAME_LEN = 18;   // 4-byte header + 12-byte payload + 2-byte CRC
constexpr uint8_t TELEMETRY_FRAME_LEN = 16; // 4-byte header + 10-byte payload + 2-byte CRC
constexpr unsigned long COMMAND_TIMEOUT_MS = 2000UL;
}

volatile bool i2cCommandReady = false;
volatile uint8_t i2cCommandFrame[nanoProto::COMMAND_FRAME_LEN] = {0};
volatile uint8_t i2cTelemetryFrame[nanoProto::TELEMETRY_FRAME_LEN] = {0};

struct NanoCommandPayload {
  uint8_t relay = 0;
  uint8_t vfdRun = 0;
  int16_t vfdFreqDeciHz = 0;
  uint8_t wellMode = 0;
  uint8_t wellIntention = 0;
  uint8_t houseMode = 0;
  uint8_t flags = 0;
  uint16_t levelFilterMs = 2000;
  uint16_t levelThreshRaw = 700;
} __attribute__((packed));

struct NanoTelemetryPayload {
  int16_t wellCurrentCentiA = 0;
  int16_t wellPressureCentiBar = 0;
  int16_t houseCurrentCentiA = 0;
  int16_t housePressureCentiBar = 0;
  uint8_t levelsMask = 0;
  uint8_t statusBits = 0;
} __attribute__((packed));

const uint8_t WELL_CURRENT_SAMPLES = 120;
const uint8_t HOUSE_PRESSURE_AVG_SAMPLES = 6;
const unsigned long LEVEL_FILTER_MS_DEFAULT = 2000;
const int LEVEL_THRESH_DEFAULT = 700;
const unsigned long LEVEL_FILTER_MS_MIN = 0;
const unsigned long LEVEL_FILTER_MS_MAX = 30000;
const int LEVEL_THRESH_MIN = 0;
const int LEVEL_THRESH_MAX = 1023;

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
  unsigned long levelFilterMs = LEVEL_FILTER_MS_DEFAULT;
  int levelThresh = LEVEL_THRESH_DEFAULT;
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
} ns;

const unsigned long ESP_STATUS_TIMEOUT_MS = 5000;
const unsigned long I2C_SANITY_CHECK_INTERVAL_MS = 5000;
const unsigned long L1_L2_STUCK_WINDOW_MS = 45000;
const unsigned long WIRING_WARNING_RATE_LIMIT_MS = 15000;

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
    default: return "UNKNOWN";
  }
}

const char* houseModeText(int mode) {
  switch (mode) {
    case 0: return "WAIT_WATER";
    case 1: return "STARTING";
    case 2: return "READY";
    case 3: return "RUNNING";
    case 4: return "STOPPING";
    case 5: return "STOPPED";
    case 6: return "FAULT";
    default: return "UNKNOWN";
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

bool readLevelPin(uint8_t pin) {
  if (pin >= A0) return analogRead(pin) > ns.levelThresh;
  return digitalRead(pin) == HIGH;
}

bool readLevelFiltered(uint8_t idx, uint8_t pin, unsigned long now) {
  bool measuredState = readLevelPin(pin);
  LevelFilter &filter = ns.levelFilters[idx];

  if (measuredState != filter.stableState) {
    if (measuredState != filter.pendingState) {
      filter.pendingState = measuredState;
      filter.pendingSince = now;
    } else if (now - filter.pendingSince >= ns.levelFilterMs) {
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

uint16_t calcCrc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x0001) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
    }
  }
  return crc;
}

void writeU16LE(uint8_t* dst, uint16_t value) {
  dst[0] = (uint8_t)(value & 0xFF);
  dst[1] = (uint8_t)((value >> 8) & 0xFF);
}

bool writeFrameBytes(uint8_t* frame, size_t frameLen, size_t offset, const void* src, size_t srcLen) {
  if (offset > frameLen || srcLen > (frameLen - offset)) return false;
  memcpy(frame + offset, src, srcLen);
  return true;
}

uint16_t readU16LE(const uint8_t* src) {
  return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

int16_t clampScaled(float value, float scale) {
  long scaled = lroundf(value * scale);
  if (scaled < -32768L) scaled = -32768L;
  if (scaled > 32767L) scaled = 32767L;
  return (int16_t)scaled;
}

volatile uint8_t nanoLastCommandSeq = 0;
volatile bool nanoCommandSeqValid = false;
unsigned long nanoLastCommandAt = 0;
unsigned long nanoCommandSeqGapCount = 0;
unsigned long nanoShortFrameCount = 0;
unsigned long nanoBadHeaderCount = 0;
unsigned long nanoBadCrcCount = 0;
unsigned long nanoParseRejectCount = 0;
unsigned long nanoLastErrorSnapshot = 0;
unsigned long nanoLastLevelTransitionAt = 0;
unsigned long nanoLastSanityCheckAt = 0;
unsigned long nanoLastWiringWarningAt = 0;
bool nanoLastL1 = false;
bool nanoLastL2 = false;

void printBootPinWarningBanner() {
  Serial.println();
  Serial.println(F("=============================================="));
  Serial.println(F("WARNING: I2C mode active (A4/A5 reserved)"));
  Serial.println(F("Level pin mapping: L1=D4, L2=D5, L3=A6, L4=A7"));
  Serial.println(F("Legacy A4/A5 level wiring is NOT compatible."));
  Serial.println(F("=============================================="));
}

void checkI2cLevelWiringSanity(unsigned long now) {
  if (now - nanoLastSanityCheckAt < I2C_SANITY_CHECK_INTERVAL_MS) return;
  nanoLastSanityCheckAt = now;

  const unsigned long errorTotal = nanoCommandSeqGapCount + nanoShortFrameCount + nanoBadHeaderCount +
                                   nanoBadCrcCount + nanoParseRejectCount;
  const unsigned long errorDelta = errorTotal - nanoLastErrorSnapshot;
  nanoLastErrorSnapshot = errorTotal;

  if (ns.levels[0] != nanoLastL1 || ns.levels[1] != nanoLastL2) {
    nanoLastLevelTransitionAt = now;
    nanoLastL1 = ns.levels[0];
    nanoLastL2 = ns.levels[1];
  }

  const bool i2cUnstable = errorDelta >= 2;
  const bool l1l2Stuck = (now - nanoLastLevelTransitionAt) >= L1_L2_STUCK_WINDOW_MS;
  if (!i2cUnstable || !l1l2Stuck) return;

  if (now - nanoLastWiringWarningAt < WIRING_WARNING_RATE_LIMIT_MS) return;
  nanoLastWiringWarningAt = now;

  Serial.println(F("[WIRING WARNING] I2C traffic unstable and L1/L2 look stuck/invalid."));
  Serial.println(F("[WIRING WARNING] Verify level sensor wiring: L1=D4, L2=D5 (NOT A4/A5)."));
  Serial.println(F("[WIRING WARNING] Legacy A4/A5 level wiring conflicts with I2C SDA/SCL."));
}

bool applyCommandFrame(const uint8_t* frame, size_t len, uint8_t protocolVersion, unsigned long now) {
  if (protocolVersion != nanoProto::VERSION || len != nanoProto::COMMAND_FRAME_LEN) {
    nanoParseRejectCount++;
    return false;
  }

  NanoCommandPayload payload;
  memcpy(&payload, frame + 4, sizeof(payload));

  ns.relayWellOn = payload.relay != 0;
  ns.vfdRun = payload.vfdRun != 0;
  ns.vfdFreqHz = payload.vfdFreqDeciHz / 10.0f;
  ns.wellMode = payload.wellMode;
  ns.wellIntention = payload.wellIntention;
  ns.houseMode = payload.houseMode;
  ns.wellAlarm = (payload.flags & nanoProto::FLAG_WELL_ALARM) != 0;
  ns.wellBlocked = (payload.flags & nanoProto::FLAG_WELL_BLOCKED) != 0;
  ns.houseAlarm = (payload.flags & nanoProto::FLAG_HOUSE_ALARM) != 0;
  ns.houseBlocked = (payload.flags & nanoProto::FLAG_HOUSE_BLOCKED) != 0;
  ns.levelFilterMs = constrain((unsigned long)payload.levelFilterMs, LEVEL_FILTER_MS_MIN, LEVEL_FILTER_MS_MAX);
  ns.levelThresh = constrain((int)payload.levelThreshRaw, LEVEL_THRESH_MIN, LEVEL_THRESH_MAX);
  ns.statusFromEsp = true;
  ns.statusUpdatedAt = now;
  nanoLastCommandAt = now;
  return true;
}

void onI2cReceive(int count) {
  if (count <= 0) return;

  uint8_t raw[nanoProto::COMMAND_FRAME_LEN] = {0};
  uint8_t idx = 0;
  while (Wire.available() && idx < sizeof(raw)) raw[idx++] = (uint8_t)Wire.read();
  while (Wire.available()) { (void)Wire.read(); idx++; }

  if (idx < nanoProto::COMMAND_FRAME_LEN) {
    nanoShortFrameCount++;
    return;
  }

  const uint8_t protocolVersion = raw[1];
  if (raw[0] != nanoProto::MAGIC || protocolVersion != nanoProto::VERSION || raw[2] != nanoProto::MSG_COMMAND) {
    nanoBadHeaderCount++;
    return;
  }

  const size_t expectedLen = nanoProto::COMMAND_FRAME_LEN;
  if (idx != expectedLen) {
    nanoShortFrameCount++;
    return;
  }

  uint16_t rxCrc = readU16LE(raw + expectedLen - 2);
  uint16_t calc = calcCrc16(raw, expectedLen - 2);
  if (rxCrc != calc) {
    nanoBadCrcCount++;
    return;
  }

  if (nanoCommandSeqValid && (uint8_t)(nanoLastCommandSeq + 1) != raw[3]) nanoCommandSeqGapCount++;
  nanoLastCommandSeq = raw[3];
  nanoCommandSeqValid = true;

  noInterrupts();
  memset((void*)i2cCommandFrame, 0, nanoProto::COMMAND_FRAME_LEN);
  memcpy((void*)i2cCommandFrame, raw, expectedLen);
  i2cCommandReady = true;
  interrupts();
}

void onI2cRequest() {
  noInterrupts();
  Wire.write((const uint8_t*)i2cTelemetryFrame, nanoProto::TELEMETRY_FRAME_LEN);
  interrupts();
}

void processEspI2c(unsigned long now) {
  if (i2cCommandReady) {
    uint8_t local[nanoProto::COMMAND_FRAME_LEN] = {0};
    noInterrupts();
    memcpy(local, (const void*)i2cCommandFrame, nanoProto::COMMAND_FRAME_LEN);
    i2cCommandReady = false;
    interrupts();

    if (!applyCommandFrame(local, nanoProto::COMMAND_FRAME_LEN, local[1], now)) {
      nanoParseRejectCount++;
    }
  }

  if (nanoLastCommandAt != 0 && (now - nanoLastCommandAt) > nanoProto::COMMAND_TIMEOUT_MS) {
    ns.relayWellOn = false;
    ns.vfdRun = false;
    ns.vfdFreqHz = 0.0f;
  }
}

void updateTelemetryFrame(unsigned long now) {
  if (now - ns.lastTelemetry < 100) return;
  ns.lastTelemetry = now;

  NanoTelemetryPayload payload;
  payload.wellCurrentCentiA = clampScaled(ns.wellCurrent, 100.0f);
  payload.wellPressureCentiBar = clampScaled(ns.wellPressureBar, 100.0f);
  payload.houseCurrentCentiA = clampScaled(ns.houseCurrent, 100.0f);
  payload.housePressureCentiBar = clampScaled(ns.housePressureBar, 100.0f);
  payload.levelsMask = (ns.levels[0] ? 1 : 0) |
                       (ns.levels[1] ? 2 : 0) |
                       (ns.levels[2] ? 4 : 0) |
                       (ns.levels[3] ? 8 : 0);
  payload.statusBits = 0;
  if (ns.vfdRun) payload.statusBits |= nanoProto::STATUS_VFD_RUN;
  if (nanoLastCommandAt != 0 && (now - nanoLastCommandAt) <= nanoProto::COMMAND_TIMEOUT_MS) {
    payload.statusBits |= nanoProto::STATUS_LINK_OK;
  } else {
    payload.statusBits |= nanoProto::STATUS_CMD_STALE;
  }

  uint8_t frame[nanoProto::TELEMETRY_FRAME_LEN] = {0};
  bool frameOk = true;
  if (nanoProto::TELEMETRY_FRAME_LEN < 6) frameOk = false;

  static uint8_t txSeq = 0;
  const uint8_t seq = txSeq++;

  if (frameOk) frame[0] = nanoProto::MAGIC;
  if (frameOk) frame[1] = nanoProto::VERSION;
  if (frameOk) frame[2] = nanoProto::MSG_TELEMETRY;
  if (frameOk) frame[3] = seq;
  if (frameOk && !writeFrameBytes(frame, nanoProto::TELEMETRY_FRAME_LEN, 4, &payload, sizeof(payload))) {
    frameOk = false;
  }

  if (!frameOk) {
    memset(frame, 0, sizeof(frame));
    frame[0] = nanoProto::MAGIC;
    frame[1] = nanoProto::VERSION;
    frame[2] = nanoProto::MSG_TELEMETRY;
    frame[3] = seq;

    const uint8_t fallbackPayload[] = {
      0, 0, 0, 0, 0, 0, 0, 0, 0, nanoProto::STATUS_CMD_STALE
    };
    (void)writeFrameBytes(frame, nanoProto::TELEMETRY_FRAME_LEN, 4, fallbackPayload, sizeof(fallbackPayload));
  }

  uint16_t crc = calcCrc16(frame, nanoProto::TELEMETRY_FRAME_LEN - 2);
  writeU16LE(frame + nanoProto::TELEMETRY_FRAME_LEN - 2, crc);

  noInterrupts();
  memcpy((void*)i2cTelemetryFrame, frame, nanoProto::TELEMETRY_FRAME_LEN);
  interrupts();
}

void sendTelemetry(unsigned long now) {
  updateTelemetryFrame(now);
}

void setup() {
  Serial.begin(9600);      // RS485 VFD
  Wire.begin(NANO_I2C_ADDRESS);
  Wire.onReceive(onI2cReceive);
  Wire.onRequest(onI2cRequest);

  pinMode(RELAY_WELL, OUTPUT);
  digitalWrite(RELAY_WELL, HIGH);

  pinMode(PIN_RS485_DE_RE, OUTPUT);
  digitalWrite(PIN_RS485_DE_RE, LOW);

  pinMode(L1, INPUT);
  pinMode(L2, INPUT);

  printBootPinWarningBanner();
  nanoLastLevelTransitionAt = millis();
  nanoLastL1 = readLevelPin(L1);
  nanoLastL2 = readLevelPin(L2);

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
  processEspI2c(now);
  feedWatchdog();
  readInputs(now);
  applyOutputs();
  feedWatchdog();
  sendTelemetry(now);
  checkI2cLevelWiringSanity(now);

  feedWatchdog();
}
