// =====================================================
// Arduino Nano I/O bridge for dual-pump project
// Nano keeps original wiring (sensors/relay/RS485 VFD),
// while all control logic is moved to ESP32.
// =====================================================

#include <SoftwareSerial.h>

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

// UART to ESP32 (SoftwareSerial to avoid conflict with RS485 on Serial)
#define ESP_RX_PIN        4
#define ESP_TX_PIN        5
SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);

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
} ns;

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
  int raw = analogRead(ACS_PIN);
  float amps = abs(raw - ns.currentZeroOffset) * (5.0f / 1023.0f) / 0.185f;
  return amps < 0.10f ? 0.0f : amps;
}

float readHouseCurrent() {
  int raw = analogRead(PIN_CURRENT);
  float voltage = raw * 5.0f / 1023.0f;
  float amps = voltage * 2.0f;
  return amps < 0.10f ? 0.0f : amps;
}

float readPressureBar(uint8_t pin) {
  int raw = analogRead(pin);
  float voltage = raw * 5.0f / 1023.0f;
  return max(0.0f, (voltage - 0.5f) * 12.0f / 4.0f);
}

bool readLevel(uint8_t pin) {
  return analogRead(pin) > 500;  // threshold for analog level channels
}

void readInputs() {
  ns.wellCurrent = readWellCurrent();
  ns.houseCurrent = readHouseCurrent();
  ns.wellPressureBar = readPressureBar(PRESSURE_PIN);
  ns.housePressureBar = readPressureBar(PIN_PRESSURE);

  ns.levels[0] = readLevel(L1);
  ns.levels[1] = readLevel(L2);
  ns.levels[2] = readLevel(L3);
  ns.levels[3] = readLevel(L4);
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
  if (now - ns.lastTelemetry < 250) return;
  ns.lastTelemetry = now;

  espSerial.print("TEL,");
  espSerial.print(now);
  espSerial.print(','); espSerial.print(ns.wellCurrent, 3);
  espSerial.print(','); espSerial.print(ns.wellPressureBar, 3);
  espSerial.print(','); espSerial.print(ns.houseCurrent, 3);
  espSerial.print(','); espSerial.print(ns.housePressureBar, 3);
  espSerial.print(','); espSerial.print(ns.levels[0] ? 1 : 0);
  espSerial.print(','); espSerial.print(ns.levels[1] ? 1 : 0);
  espSerial.print(','); espSerial.print(ns.levels[2] ? 1 : 0);
  espSerial.print(','); espSerial.print(ns.levels[3] ? 1 : 0);
  espSerial.print(','); espSerial.print(ns.vfdRun ? 1 : 0);
  espSerial.print(','); espSerial.print(ns.vfdFreqHz, 1);
  espSerial.println();
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
    delay(2);
  }
  ns.currentZeroOffset = sum / 600.0f;
  if (ns.currentZeroOffset < 400 || ns.currentZeroOffset > 600) ns.currentZeroOffset = 512.0f;

  initVFD();
}

void loop() {
  unsigned long now = millis();
  processEspUart();
  readInputs();
  applyOutputs();
  sendTelemetry(now);
}
