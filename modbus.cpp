#include "modbus.h"

#include "pins_config.h"

HardwareSerial VfdSerial(2);

namespace {
void rs485Transmit() {
  digitalWrite(RS485_DE_PIN, HIGH);
  delayMicroseconds(120);
}

void rs485Receive() {
  digitalWrite(RS485_DE_PIN, LOW);
  delayMicroseconds(120);
}

size_t readWithTimeout(uint8_t* out, size_t needed, uint32_t timeoutMs) {
  size_t got = 0;
  unsigned long start = millis();
  while (got < needed && (millis() - start) < timeoutMs) {
    while (VfdSerial.available() > 0 && got < needed) {
      out[got++] = (uint8_t)VfdSerial.read();
    }
    delay(1);
  }
  return got;
}
}  // namespace

void modbusInit() {
  pinMode(RS485_DE_PIN, OUTPUT);
  rs485Receive();
  VfdSerial.begin(VFD_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
}

uint16_t modbusCRC(const uint8_t* data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

bool vfdWriteReg(uint16_t reg, uint16_t value, uint32_t timeoutMs) {
  uint8_t req[8] = {VFD_SLAVE_ID, 0x06, (uint8_t)(reg >> 8), (uint8_t)reg, (uint8_t)(value >> 8), (uint8_t)value, 0, 0};
  uint16_t crc = modbusCRC(req, 6);
  req[6] = (uint8_t)crc;
  req[7] = (uint8_t)(crc >> 8);

  while (VfdSerial.available()) VfdSerial.read();
  rs485Transmit();
  VfdSerial.write(req, sizeof(req));
  VfdSerial.flush();
  rs485Receive();

  uint8_t resp[8] = {0};
  size_t got = readWithTimeout(resp, sizeof(resp), timeoutMs);
  if (got != sizeof(resp)) return false;
  uint16_t rcrc = (uint16_t)resp[7] << 8 | resp[6];
  return rcrc == modbusCRC(resp, 6);
}

bool vfdReadReg(uint16_t reg, uint16_t& out, uint32_t timeoutMs) {
  uint8_t req[8] = {VFD_SLAVE_ID, 0x03, (uint8_t)(reg >> 8), (uint8_t)reg, 0x00, 0x01, 0, 0};
  uint16_t crc = modbusCRC(req, 6);
  req[6] = (uint8_t)crc;
  req[7] = (uint8_t)(crc >> 8);

  while (VfdSerial.available()) VfdSerial.read();
  rs485Transmit();
  VfdSerial.write(req, sizeof(req));
  VfdSerial.flush();
  rs485Receive();

  uint8_t resp[7] = {0};
  size_t got = readWithTimeout(resp, sizeof(resp), timeoutMs);
  if (got != sizeof(resp) || resp[0] != VFD_SLAVE_ID || resp[1] != 0x03 || resp[2] != 0x02) return false;
  uint16_t rcrc = (uint16_t)resp[6] << 8 | resp[5];
  if (rcrc != modbusCRC(resp, 5)) return false;
  out = (uint16_t)resp[3] << 8 | resp[4];
  return true;
}
