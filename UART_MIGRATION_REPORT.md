# UART Migration Report (Nano <-> ESP32)

## Why I2C was removed

I2C link was unstable in real hardware despite multiple fixes and diagnostics. After moving RS-485/Modbus responsibilities to ESP32, Nano workload became lighter, so UART became the preferred robust controller-to-controller transport.

## Chosen UART pins

- ESP32 controller link (`Serial2`):
  - RX: GPIO16
  - TX: GPIO17
- Nano controller link (`SoftwareSerial`):
  - TX: D10
  - RX: D11

## Electrical notes

- Nano TX (5V) -> ESP32 RX (3.3V) via resistor divider:
  - 10k series from Nano TX to ESP32 RX node
  - 20k from ESP32 RX node to GND
- ESP32 TX -> Nano RX direct
- Common ground required between Nano and ESP32

## Packet structure

Framed binary protocol is retained and extended with explicit payload length:

1. magic byte 0 (`0xA5`)
2. magic byte 1 (`0x5A`)
3. protocol version (`v3`)
4. message type (`command` / `telemetry`)
5. sequence number
6. payload length
7. payload
8. CRC16 (Modbus polynomial)

## Request-response timing

- Link speed: `38400` baud
- Command period: ~`100 ms`
- ESP32 sends command frame, then waits for telemetry response (bounded timeout)
- Telemetry remains compact and semantically equivalent to previous implementation

## Timeout / link rules

- ESP32 stale telemetry timeout preserved (`TELEMETRY_STALE_MS`)
- If telemetry is stale/missing, ESP32 marks link lost and enforces safe stop
- Nano command timeout preserved (`COMMAND_TIMEOUT_MS`)
- If commands become stale on Nano, local relay output is forced to safe state (off)

## Safe-state behavior

- ESP32 fail-safe behavior unchanged in philosophy:
  - link degraded/lost -> pumps stopped
  - state forced to safe modes until valid link is restored
- Nano fail-safe behavior unchanged in philosophy:
  - no fresh valid command -> disable relay output

## What remained unchanged

- Pump algorithms and state machines for well/house control
- Protection thresholds and protections flow
- Telemetry engineering meaning:
  - well current/pressure
  - house pressure
  - levels L1/L2/L3/L4
  - status bits and sequence tracking
- Event logging retained (with numeric sanitization added to avoid bogus overflow-like values in event payloads)
