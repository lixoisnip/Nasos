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
constexpr uint8_t AVG_SAMPLES_DEFAULT = 12;
constexpr int LEVEL_HI = 540;
constexpr int LEVEL_LO = 480;

struct NanoState {
  float wellCurrent = 0.0f;
  float wellPressureBar = 0.0f;
  float housePressureBar = 0.0f;
  bool levels[4] = {false, false, false, false};
  bool relayWellOn = false;
  bool errorFlag = false;
  float currentZeroOffset = 512.0f;
  uint8_t avgSamples = AVG_SAMPLES_DEFAULT;
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

int readAveragedAnalog(uint8_t pin, uint8_t samples) {
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  return (int)(sum / samples);
}

float readWellCurrent() {
  int raw = readAveragedAnalog(ACS_PIN, ns.avgSamples);
  float amps = abs(raw - ns.currentZeroOffset) * (5.0f / 1023.0f) / 0.185f;
  return amps < 0.10f ? 0.0f : amps;
}

float readPressureBar(uint8_t pin) {
  int raw = readAveragedAnalog(pin, ns.avgSamples);
  float voltage = raw * 5.0f / 1023.0f;
  return max(0.0f, (voltage - 0.5f) * 12.0f / 4.0f);
}

bool readLevelWithHysteresis(uint8_t pin, bool prev) {
  int raw = readAveragedAnalog(pin, 4);
  if (prev) return raw > LEVEL_LO;
  return raw > LEVEL_HI;
}

void readInputs() {
  ns.wellCurrent = readWellCurrent();
  ns.wellPressureBar = readPressureBar(PRESSURE_PIN);
  ns.housePressureBar = readPressureBar(PIN_PRESSURE);
  ns.levels[0] = readLevelWithHysteresis(L1, ns.levels[0]);
  ns.levels[1] = readLevelWithHysteresis(L2, ns.levels[1]);
  ns.levels[2] = readLevelWithHysteresis(L3, ns.levels[2]);
  ns.levels[3] = readLevelWithHysteresis(L4, ns.levels[3]);
}

void applyOutputs() {
  digitalWrite(RELAY_WELL, ns.relayWellOn ? LOW : HIGH);
}

void calibrateZero() {
  long sum = 0;
  for (int i = 0; i < 300; i++) {
    sum += analogRead(ACS_PIN);
    delay(2);
  }
  ns.currentZeroOffset = sum / 300.0f;
}

void onI2CReceive(int len) {
  if (len < 1) {
    while (Wire.available()) Wire.read();
    return;
  }
  char cmd = Wire.read();
  uint8_t value = Wire.available() ? Wire.read() : 0;

  if (cmd == 'C') {
    ns.relayWellOn = value == 1;
  } else if (cmd == 'Z') {
    calibrateZero();
  } else if (cmd == 'R') {
    ns.errorFlag = false;
  } else if (cmd == 'N') {
    ns.avgSamples = constrain(value, 4, 20);
  }

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
  calibrateZero();

  Wire.begin(NANO_I2C_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
}

void loop() {
  readInputs();
  applyOutputs();
  delay(20);
}
