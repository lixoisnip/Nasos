# UART + RS-485 Migration Report

## 1) Removal of I2C controller link

Controller-to-controller transport between ESP32 and Nano is migrated from I2C to UART framed packets.
I2C is no longer used for ESP32<->Nano command/telemetry exchange.

## 2) UART transport choice

- Transport: UART request-response
- Default baud: **38400** (conservative and stable for Nano SoftwareSerial + ESP32)
- Frame philosophy preserved:
  - magic bytes,
  - version,
  - message type,
  - sequence number,
  - payload length,
  - payload,
  - CRC16.

Flow:
1. ESP32 sends command frame to Nano.
2. Nano validates/applies outputs and config.
3. Nano returns telemetry frame.

## 3) Final pinout

### ESP32
- Nano UART RX: GPIO16
- Nano UART TX: GPIO17
- MAX485 TX/DI: GPIO25
- MAX485 RX/RO: GPIO26
- MAX485 DE/RE: GPIO27
- TFT_CS: GPIO5
- TFT_DC: GPIO2
- TFT_RST: GPIO4

### Arduino Nano
- UART TX to ESP32: D10
- UART RX from ESP32: D11

## 4) Final role of Nano

Nano is a low-level deterministic I/O board only:
- read level inputs,
- read analog channels,
- control relays,
- keep simple filtered telemetry,
- receive command frames from ESP32,
- send telemetry frames back to ESP32.

Nano no longer owns RS-485/Modbus and no longer owns display logic.

## 5) Final role of ESP32

ESP32 is the main controller:
- well/house logic and protections,
- web UI/API/settings/logs,
- display handling,
- Nano command/telemetry master over UART,
- RS-485/Modbus RTU master for VFD.

ESP32 uses separate UART resources:
- one UART for Nano link,
- one UART for RS-485/Modbus.

## 6) RS-485 moved to ESP32

MAX485 is wired to ESP32 only (TX=GPIO25, RX=GPIO26, DE/RE=GPIO27).
VFD Modbus transactions are executed on ESP32.
Nano firmware contains no VFD Modbus handling.

## 7) Electrical assumptions / level shifting

- Nano TX (5V) to ESP32 RX (3.3V): mandatory 10k/20k divider.
- MAX485 RO to ESP32 RX: use 10k/20k divider if module RO is 5V TTL.
- ESP32 TX to Nano RX and ESP32 TX to MAX485 DI can be direct.
- Common GND across Nano, ESP32 and RS-485 hardware is required.
