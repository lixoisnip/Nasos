#pragma once

#include <Arduino.h>

void modbusInit();
uint16_t modbusCRC(const uint8_t* data, uint8_t len);
bool vfdWriteReg(uint16_t reg, uint16_t value, uint32_t timeoutMs = 120);
bool vfdReadReg(uint16_t reg, uint16_t& out, uint32_t timeoutMs = 120);
