#pragma once

#include <Arduino.h>

constexpr uint8_t NANO_I2C_ADDR = 0x10;
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint32_t I2C_CLOCK = 100000;
constexpr unsigned long NANO_POLL_MS = 120;

constexpr int RS485_TX_PIN = 17;
constexpr int RS485_RX_PIN = 16;
constexpr int RS485_DE_PIN = 27;
constexpr uint32_t VFD_BAUD = 9600;
constexpr uint8_t VFD_SLAVE_ID = 0x08;
constexpr uint16_t VFD_REG_RUN_CMD = 0x9CA7;
constexpr uint16_t VFD_REG_FREQ_CMD = 0x9CA6;
constexpr uint16_t VFD_REG_OUT_FREQ_FB = 0x9CAA;
constexpr uint16_t VFD_REG_OUT_CURRENT_FB = 0x9CAB;
constexpr float VFD_FREQ_SCALE = 0.01f;
constexpr float VFD_CURRENT_SCALE = 0.01f;
constexpr unsigned long VFD_POLL_MS = 150;
constexpr unsigned long STARTUP_NO_CURRENT_DELAY_MS = 2500;
constexpr float HOUSE_MAX_PRESSURE_TRIP_BAR = 2.2f;
