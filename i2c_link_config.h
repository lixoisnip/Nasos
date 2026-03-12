#pragma once

#include <Arduino.h>

namespace i2cLinkCfg {
// Shared Nano<->ESP32 I2C link configuration.
// Hardware-validated split architecture address (Nano slave / ESP32 master target).
constexpr uint8_t NANO_SLAVE_ADDRESS = 0x09;
constexpr uint32_t BUS_FREQUENCY_HZ = 100000UL;

// Default ESP32 pins for this project wiring.
constexpr int ESP32_SDA_PIN = 21;
constexpr int ESP32_SCL_PIN = 22;

constexpr const char* NANO_FW_NAME = "nasos-nano-bridge";
constexpr const char* NANO_FW_VERSION = "1.3.0";
}  // namespace i2cLinkCfg
