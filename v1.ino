// =====================================================
// Arduino Nano I/O bridge for dual-pump project
// Nano keeps original wiring for sensors and relay only,
// while all control logic + VFD RS485 is moved to ESP32.
// =====================================================

#include <SoftwareSerial.h>
#include <avr/wdt.h>
#include <string.h>
#include <stdlib.h>

// Original project pins (unchanged wiring)
#define RELAY_WELL        3
#define ACS_PIN           A1
#define PIN_CURRENT       A0
#define PRESSURE_PIN      A2
#define PIN_PRESSURE      A3
#define L1                4
#define L2                5
#define L3                A6
#define L4                A7

// Nano display logic is intentionally removed.
// Display belongs to ESP32 only.

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

// UART link to ESP32 (Nano side uses SoftwareSerial to keep USB Serial debug)
constexpr uint8_t NANO_UART_RX_PIN = 11;
constexpr uint8_t NANO_UART_TX_PIN = 10;
constexpr unsigned long NANO_UART_BAUD = 38400UL;
SoftwareSerial controllerSerial(NANO_UART_RX_PIN, NANO_UART_TX_PIN);

namespace nanoProto {
constexpr uint8_t MAGIC_0 = 0xA5;
constexpr uint8_t MAGIC_1 = 0x5A;
constexpr uint8_t VERSION = 3;
constexpr uint8_t MSG_COMMAND = 1;
constexpr uint8_t MSG_TELEMETRY = 2;
constexpr uint8_t FLAG_WELL_ALARM = 1 << 0;
constexpr uint8_t FLAG_WELL_BLOCKED = 1 << 1;
constexpr uint8_t FLAG_HOUSE_ALARM = 1 << 2;
constexpr uint8_t FLAG_HOUSE_BLOCKED = 1 << 3;
constexpr uint8_t STATUS_CMD_STALE = 1 << 0;
constexpr uint8_t STATUS_LINK_OK = 1 << 1;
constexpr uint8_t STATUS_RELAY_WELL_ACTIVE = 1 << 2;
constexpr uint8_t COMMAND_PAYLOAD_LEN = 12;
constexpr uint8_t TELEMETRY_PAYLOAD_LEN = 10;
constexpr uint8_t HEADER_LEN = 6; // magic0, magic1, version, type, seq, payloadLen
constexpr uint8_t CRC_LEN = 2;
constexpr uint8_t COMMAND_FRAME_LEN = HEADER_LEN + COMMAND_PAYLOAD_LEN + CRC_LEN;
constexpr uint8_t TELEMETRY_FRAME_LEN = HEADER_LEN + TELEMETRY_PAYLOAD_LEN + CRC_LEN;
constexpr unsigned long COMMAND_TIMEOUT_MS = 2000UL;
}

struct NanoCommandPayload {
  uint8_t relay = 0;
  uint8_t wellMode = 0;
  uint8_t wellIntention = 0;
  uint8_t houseMode = 0;
  uint8_t flags = 0;
  uint16_t levelFilterMs = 2000;
  uint16_t levelThreshRaw = 700;
  uint8_t reserved[4] = {0, 0, 0, 0};
} __attribute__((packed));

struct NanoTelemetryPayload {
  int16_t wellCurrentCentiA = 0;
  int16_t wellPressureCentiBar = 0;
  int16_t analogAuxRaw = 0;
  int16_t housePressureCentiBar = 0;
  uint8_t levelsMask = 0;
  uint8_t statusBits = 0;
} __attribute__((packed));

const uint8_t WELL_CURRENT_SAMPLES = 120;
const uint8_t HOUSE_CURRENT_AVG_SAMPLES = 12;
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
const unsigned long UART_SANITY_CHECK_INTERVAL_MS = 5000;
const unsigned long L1_L2_STUCK_WINDOW_MS = 45000;
const unsigned long WIRING_WARNING_RATE_LIMIT_MS = 15000;
const unsigned long NANO_HEARTBEAT_INTERVAL_MS = 5000;

const float WELL_CURRENT_DRY = 3.3f;
const float WELL_CURRENT_OVERLOAD = 4.3f;
const float WELL_PRESSURE_WARNING = 1.2f;
const float WELL_PRESSURE_BLOCK = 1.5f;

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

float readAuxAnalog() {
  unsigned int sum = 0;
  for (uint8_t i = 0; i < HOUSE_CURRENT_AVG_SAMPLES; i++) {
    sum += (unsigned int)analogRead(PIN_CURRENT);
  }
  return (float)sum / HOUSE_CURRENT_AVG_SAMPLES;
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
  ns.houseCurrent = readAuxAnalog();
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
  Serial.println(F("WARNING: UART mode active (A4/A5 free)"));
  Serial.println(F("Level pin mapping: L1=D4, L2=D5, L3=A6, L4=A7"));
  Serial.println(F("Nano link: SoftwareSerial RX=D11 TX=D10 @38400"));
  Serial.println(F("=============================================="));
}

void checkUartLevelWiringSanity(unsigned long now) {
  if (now - nanoLastSanityCheckAt < UART_SANITY_CHECK_INTERVAL_MS) return;
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

  const bool uartUnstable = errorDelta >= 2;
  const bool l1l2Stuck = (now - nanoLastLevelTransitionAt) >= L1_L2_STUCK_WINDOW_MS;
  if (!uartUnstable || !l1l2Stuck) return;

  if (now - nanoLastWiringWarningAt < WIRING_WARNING_RATE_LIMIT_MS) return;
  nanoLastWiringWarningAt = now;

  Serial.println(F("[WIRING WARNING] UART traffic unstable and L1/L2 look stuck/invalid."));
  Serial.println(F("[WIRING WARNING] Verify level sensor wiring: L1=D4, L2=D5 (NOT A4/A5)."));
  Serial.println(F("[WIRING WARNING] Legacy A4/A5 level wiring from old I2C layout may be stale."));
}

bool applyCommandFrame(const uint8_t* frame, size_t len, uint8_t protocolVersion, unsigned long now) {
  if (protocolVersion != nanoProto::VERSION || len != nanoProto::COMMAND_FRAME_LEN) {
    nanoParseRejectCount++;
    return false;
  }

  NanoCommandPayload payload;
  memcpy(&payload, frame + nanoProto::HEADER_LEN, sizeof(payload));

  ns.relayWellOn = payload.relay != 0;
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

bool parseCommandFrameFromUart(unsigned long now, uint8_t* rxFrame, uint8_t& idx) {
  while (controllerSerial.available()) {
    const uint8_t b = (uint8_t)controllerSerial.read();
    if (idx == 0 && b != nanoProto::MAGIC_0) continue;
    if (idx == 1 && b != nanoProto::MAGIC_1) {
      idx = 0;
      if (b == nanoProto::MAGIC_0) rxFrame[idx++] = b;
      continue;
    }

    rxFrame[idx++] = b;
    if (idx == nanoProto::HEADER_LEN) {
      if (rxFrame[2] != nanoProto::VERSION || rxFrame[3] != nanoProto::MSG_COMMAND || rxFrame[5] != nanoProto::COMMAND_PAYLOAD_LEN) {
        nanoBadHeaderCount++;
        idx = 0;
      }
    }

    if (idx >= nanoProto::COMMAND_FRAME_LEN) {
      const uint16_t rxCrc = readU16LE(rxFrame + nanoProto::COMMAND_FRAME_LEN - 2);
      const uint16_t calc = calcCrc16(rxFrame, nanoProto::COMMAND_FRAME_LEN - 2);
      if (rxCrc != calc) {
        nanoBadCrcCount++;
        idx = 0;
        return false;
      }

      const uint8_t seq = rxFrame[4];
      if (nanoCommandSeqValid && (uint8_t)(nanoLastCommandSeq + 1) != seq) nanoCommandSeqGapCount++;
      nanoLastCommandSeq = seq;
      nanoCommandSeqValid = true;

      if (!applyCommandFrame(rxFrame, nanoProto::COMMAND_FRAME_LEN, rxFrame[2], now)) {
        nanoParseRejectCount++;
        idx = 0;
        return false;
      }
      idx = 0;
      return true;
    }

  }
  return false;
}

void processEspUart(unsigned long now) {
  static uint8_t rxFrame[nanoProto::COMMAND_FRAME_LEN] = {0};
  static uint8_t rxIdx = 0;
  const bool commandAccepted = parseCommandFrameFromUart(now, rxFrame, rxIdx);

  if (nanoLastCommandAt != 0 && (now - nanoLastCommandAt) > nanoProto::COMMAND_TIMEOUT_MS) {
    ns.relayWellOn = false;
  }

  if (commandAccepted) {
    sendTelemetry(now);
  }
}

void updateTelemetryFrame(unsigned long now) {
  if (now - ns.lastTelemetry < 100) return;
  ns.lastTelemetry = now;

  NanoTelemetryPayload payload;
  payload.wellCurrentCentiA = clampScaled(ns.wellCurrent, 100.0f);
  payload.wellPressureCentiBar = clampScaled(ns.wellPressureBar, 100.0f);
  payload.analogAuxRaw = (int16_t)constrain((int)lroundf(ns.houseCurrent), -32768, 32767);
  payload.housePressureCentiBar = clampScaled(ns.housePressureBar, 100.0f);
  payload.levelsMask = (ns.levels[0] ? 1 : 0) |
                       (ns.levels[1] ? 2 : 0) |
                       (ns.levels[2] ? 4 : 0) |
                       (ns.levels[3] ? 8 : 0);
  payload.statusBits = 0;
  if (ns.relayWellOn) payload.statusBits |= nanoProto::STATUS_RELAY_WELL_ACTIVE;
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

  if (frameOk) frame[0] = nanoProto::MAGIC_0;
  if (frameOk) frame[1] = nanoProto::MAGIC_1;
  if (frameOk) frame[2] = nanoProto::VERSION;
  if (frameOk) frame[3] = nanoProto::MSG_TELEMETRY;
  if (frameOk) frame[4] = seq;
  if (frameOk) frame[5] = nanoProto::TELEMETRY_PAYLOAD_LEN;
  if (frameOk && !writeFrameBytes(frame, nanoProto::TELEMETRY_FRAME_LEN, nanoProto::HEADER_LEN, &payload, sizeof(payload))) {
    frameOk = false;
  }

  if (!frameOk) {
    memset(frame, 0, sizeof(frame));
    frame[0] = nanoProto::MAGIC_0;
    frame[1] = nanoProto::MAGIC_1;
    frame[2] = nanoProto::VERSION;
    frame[3] = nanoProto::MSG_TELEMETRY;
    frame[4] = seq;
    frame[5] = nanoProto::TELEMETRY_PAYLOAD_LEN;

    const uint8_t fallbackPayload[] = {
      0, 0, 0, 0, 0, 0, 0, 0, 0, nanoProto::STATUS_CMD_STALE
    };
    (void)writeFrameBytes(frame, nanoProto::TELEMETRY_FRAME_LEN, nanoProto::HEADER_LEN, fallbackPayload, sizeof(fallbackPayload));
  }

  uint16_t crc = calcCrc16(frame, nanoProto::TELEMETRY_FRAME_LEN - 2);
  writeU16LE(frame + nanoProto::TELEMETRY_FRAME_LEN - 2, crc);

  controllerSerial.write(frame, nanoProto::TELEMETRY_FRAME_LEN);
}

void sendTelemetry(unsigned long now) {
  updateTelemetryFrame(now);
}

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.println(F("[BOOT] Nano firmware startup"));
  Serial.println(F("[BOOT] Firmware: nasos-nano-bridge-uart v2.0.0"));
  controllerSerial.begin(NANO_UART_BAUD);
  Serial.print(F("[BOOT] Controller UART ready RX="));
  Serial.print(NANO_UART_RX_PIN);
  Serial.print(F(" TX="));
  Serial.print(NANO_UART_TX_PIN);
  Serial.print(F(" baud="));
  Serial.println(NANO_UART_BAUD);

  pinMode(RELAY_WELL, OUTPUT);
  digitalWrite(RELAY_WELL, HIGH);

  pinMode(L1, INPUT);
  pinMode(L2, INPUT);

  printBootPinWarningBanner();
  nanoLastLevelTransitionAt = millis();
  nanoLastL1 = readLevelPin(L1);
  nanoLastL2 = readLevelPin(L2);

  Serial.println(F("[BOOT] Calibrating current zero offset (600 samples)..."));
  long sum = 0;
  for (int i = 0; i < 600; i++) {
    sum += analogRead(ACS_PIN);
    if ((i % 50) == 0) feedWatchdog();
    delay(2);
  }
  ns.currentZeroOffset = sum / 600.0f;
  if (ns.currentZeroOffset < 400 || ns.currentZeroOffset > 600) ns.currentZeroOffset = 512.0f;
  Serial.print(F("[BOOT] Current zero offset="));
  Serial.println(ns.currentZeroOffset);

  wdt_enable(WDTO_4S);
  feedWatchdog();
  Serial.println(F("[BOOT] Nano setup complete"));
}

void loop() {
  feedWatchdog();
  unsigned long now = millis();
  static unsigned long lastHeartbeatMs = 0;
  if (now - lastHeartbeatMs >= NANO_HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = now;
    Serial.print(F("[HEARTBEAT] Nano alive; UART seqValid="));
    Serial.print(nanoCommandSeqValid ? 1 : 0);
    Serial.print(F(", cmdAgeMs="));
    Serial.println(nanoLastCommandAt ? (now - nanoLastCommandAt) : 0UL);
  }
  processEspUart(now);
  feedWatchdog();
  readInputs(now);
  applyOutputs();
  feedWatchdog();
  checkUartLevelWiringSanity(now);

  feedWatchdog();
}
