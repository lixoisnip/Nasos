#include <cassert>
#include <cstdint>
#include <vector>

static uint16_t modbusCRC(const uint8_t* data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

static int avg(const std::vector<int>& v) {
  long s = 0;
  for (int x : v) s += x;
  return (int)(s / v.size());
}

int main() {
  const uint8_t req[] = {0x08, 0x03, 0x9C, 0xAA, 0x00, 0x01};
  assert(modbusCRC(req, sizeof(req)) == 0xE38A);

  std::vector<int> samples = {500, 505, 510, 495, 490, 500, 512, 508, 499, 501};
  int a = avg(samples);
  assert(a >= 500 && a <= 502);
  return 0;
}
