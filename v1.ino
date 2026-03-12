// =====================================================
// Arduino Nano I/O bridge for dual-pump project
// Nano only reads local inputs and drives the well relay.
// VFD Modbus/RS-485 is handled by ESP32.
// =====================================================

#include <Wire.h>

#define RELAY_WELL     3
#define ACS_PIN        A1
#define PRESSURE_PIN   A2
#define PIN_PRESSURE   A3
#define L1             A4
#define L2             A5
#define L3             A6
#define L4             A7

constexpr uint8_t NANO_I2C_ADDR = 0x10;

struct NanoState {
  float wellCurrent = 0.0f;
  float wellPressureBar = 0.0f;
  float housePressureBar = 0.0f;
  bool levels[4] = {false, false, false, false};
  bool relayWellOn = false;
  float currentZeroOffset = 512.0f;
} ns;

#pragma pack(push, 1)
struct NanoFrame {
  uint8_t magic;
  uint32_t ms;
  int16_t wellCurrent_cA;
  int16_t wellPressure_cBar;
  int16_t housePressure_cBar;
  uint8_t levelsMask;
  uint8_t relayState;
  uint8_t crc;
};
#pragma pack(pop)

uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) crc ^= data[i];
  return crc;
}

float readWellCurrent() {
  int raw = analogRead(ACS_PIN);
  float amps = abs(raw - ns.currentZeroOffset) * (5.0f / 1023.0f) / 0.185f;
  return amps < 0.10f ? 0.0f : amps;
}

float readPressureBar(uint8_t pin) {
  int raw = analogRead(pin);
  float voltage = raw * 5.0f / 1023.0f;
  return max(0.0f, (voltage - 0.5f) * 12.0f / 4.0f);
}

bool readLevel(uint8_t pin) {
  return analogRead(pin) > 500;
}

void readInputs() {
  ns.wellCurrent = readWellCurrent();
  ns.wellPressureBar = readPressureBar(PRESSURE_PIN);
  ns.housePressureBar = readPressureBar(PIN_PRESSURE);
  ns.levels[0] = readLevel(L1);
  ns.levels[1] = readLevel(L2);
  ns.levels[2] = readLevel(L3);
  ns.levels[3] = readLevel(L4);
}

void applyOutputs() {
  digitalWrite(RELAY_WELL, ns.relayWellOn ? LOW : HIGH);  // active LOW relay
}

void onI2CReceive(int len) {
  if (len < 2) {
    while (Wire.available()) Wire.read();
    return;
  }
  char cmd = Wire.read();
  uint8_t value = Wire.read();
  if (cmd == 'C') ns.relayWellOn = value == 1;
  while (Wire.available()) Wire.read();
}

void onI2CRequest() {
  NanoFrame f{};
  f.magic = 0xA5;
  f.ms = millis();
  f.wellCurrent_cA = (int16_t)(ns.wellCurrent * 100.0f);
  f.wellPressure_cBar = (int16_t)(ns.wellPressureBar * 100.0f);
  f.housePressure_cBar = (int16_t)(ns.housePressureBar * 100.0f);
  f.levelsMask = (ns.levels[0] ? 0x01 : 0) |
                 (ns.levels[1] ? 0x02 : 0) |
                 (ns.levels[2] ? 0x04 : 0) |
                 (ns.levels[3] ? 0x08 : 0);
  f.relayState = ns.relayWellOn ? 1 : 0;
  f.crc = crc8(reinterpret_cast<uint8_t*>(&f), sizeof(NanoFrame) - 1);
  Wire.write(reinterpret_cast<uint8_t*>(&f), sizeof(NanoFrame));
}

void setup() {
  pinMode(RELAY_WELL, OUTPUT);
  digitalWrite(RELAY_WELL, HIGH);

  long sum = 0;
  for (int i = 0; i < 600; i++) {
    sum += analogRead(ACS_PIN);
    delay(2);
  }
  ns.currentZeroOffset = sum / 600.0f;
  if (ns.currentZeroOffset < 400 || ns.currentZeroOffset > 600) ns.currentZeroOffset = 512.0f;

  Wire.begin(NANO_I2C_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
}

void loop() {
  readInputs();
  applyOutputs();
  delay(20);
}
