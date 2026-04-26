#include "telemetry.h"

#include <Wire.h>

#include "modbus.h"
#include "pins_config.h"

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

namespace {
Telemetry* tm = nullptr;
Controller* st = nullptr;

uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) crc ^= data[i];
  return crc;
}
}  // namespace

void telemetryInit(Telemetry* tmPtr, Controller* stPtr) {
  tm = tmPtr;
  st = stPtr;
}

void pollNano(unsigned long now) {
  static unsigned long last = 0;
  static unsigned long lastOk = 0;
  if (now - last < NANO_POLL_MS) return;
  last = now;

  Wire.requestFrom((int)NANO_I2C_ADDR, (int)sizeof(NanoFrame));
  if (Wire.available() != (int)sizeof(NanoFrame)) {
    tm->nanoOnline = (now - lastOk) < 2000;
    return;
  }

  NanoFrame f{};
  uint8_t* p = reinterpret_cast<uint8_t*>(&f);
  for (size_t i = 0; i < sizeof(NanoFrame); ++i) p[i] = Wire.read();
  if (f.magic != 0xA5 || crc8(p, sizeof(NanoFrame) - 1) != f.crc) return;

  tm->ts = f.ms;
  tm->wellCurrent = f.wellCurrent_cA / 100.0f;
  tm->wellPressure = f.wellPressure_cBar / 100.0f;
  tm->housePressure = f.housePressure_cBar / 100.0f;
  tm->levels[0] = f.levelsMask & 0x01;
  tm->levels[1] = f.levelsMask & 0x02;
  tm->levels[2] = f.levelsMask & 0x04;
  tm->levels[3] = f.levelsMask & 0x08;
  tm->wellRelayFeedback = f.relayState;
  tm->valid = true;
  tm->nanoOnline = true;
  lastOk = now;
}

void sendNanoCommand() {
  Wire.beginTransmission(NANO_I2C_ADDR);
  Wire.write('C');
  Wire.write(st->wellRelay ? 1 : 0);
  Wire.endTransmission();
}

void pollVfd(unsigned long now) {
  static unsigned long last = 0;
  static unsigned long lastOk = 0;
  if (now - last < VFD_POLL_MS) return;
  last = now;

  uint16_t raw = 0;
  bool okCurrent = vfdReadReg(VFD_REG_OUT_CURRENT_FB, raw);
  if (okCurrent) tm->houseCurrent = raw * VFD_CURRENT_SCALE;

  bool okFreq = vfdReadReg(VFD_REG_OUT_FREQ_FB, raw);
  if (okFreq) tm->vfdFreqFeedback = raw * VFD_FREQ_SCALE;

  if (okCurrent || okFreq) {
    tm->vfdOnline = true;
    lastOk = now;
  } else {
    tm->vfdOnline = (now - lastOk) < 3000;
  }

  tm->vfdRunFeedback = tm->vfdFreqFeedback > 0.5f;
}
