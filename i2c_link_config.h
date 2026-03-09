#pragma once

#include <Arduino.h>

namespace i2cLinkCfg {
// Shared Nano<->ESP32 I2C link configuration.
constexpr uint8_t NANO_SLAVE_ADDRESS = 0x2A;
constexpr uint32_t BUS_FREQUENCY_HZ = 100000UL;

// Default ESP32 pins for this project wiring.
constexpr int ESP32_SDA_PIN = 21;
constexpr int ESP32_SCL_PIN = 22;

constexpr const char* NANO_FW_NAME = "nasos-nano-bridge";
constexpr const char* NANO_FW_VERSION = "1.1.0";
}  // namespace i2cLinkCfg
