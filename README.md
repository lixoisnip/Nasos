# Nasos: ESP32 main controller + Arduino Nano I/O board

## Firmware layout

- **ESP32 main firmware:** `esp32_controller.ino`
- **Arduino Nano I/O firmware:** `v1.ino`
- `Osnova.ino` remains historical reference only.

## Final architecture

- **ESP32** is the main controller:
  - full pump logic/protections,
  - web UI/API/settings/logging,
  - display handling,
  - RS-485/Modbus with VFD,
  - command/telemetry master for Nano.
- **Arduino Nano** is only a low-level I/O executor:
  - reads levels and analog channels,
  - controls relay outputs,
  - applies simple filtering,
  - executes commands from ESP32,
  - replies with telemetry frames over UART.

## Final pinout and wiring

### ESP32

- Nano UART RX: **GPIO16**
- Nano UART TX: **GPIO17**
- MAX485 TX/DI: **GPIO25**
- MAX485 RX/RO: **GPIO26**
- MAX485 DE/RE: **GPIO27**
- TFT_CS: **GPIO5**
- TFT_DC: **GPIO2**
- TFT_RST: **GPIO4**

### Nano

- UART TX to ESP32: **D10**
- UART RX from ESP32: **D11**

### Electrical requirements

- **Nano TX (5V) -> ESP32 RX (3.3V) requires resistor divider:**
  - Nano TX -> `10k` -> ESP32 RX node
  - ESP32 RX node -> `20k` -> GND
- **MAX485 RO -> ESP32 RX also requires divider `10k/20k` if RO is 5V logic:**
  - MAX485 RO -> `10k` -> ESP32 RX node
  - ESP32 RX node -> `20k` -> GND
- ESP32 TX -> Nano RX can be direct.
- ESP32 TX -> MAX485 DI can be direct.
- MAX485 `DE` and `RE` are tied together and controlled by ESP32 GPIO27.
- **Common GND between ESP32, Nano, and MAX485 is mandatory.**

## Controller communication (Nano <-> ESP32)

- I2C controller-to-controller link is removed.
- UART binary framing is used (conservative baud: **38400**).
- Request-response flow:
  - ESP32 sends command frame,
  - Nano validates/applies it,
  - Nano replies with telemetry frame.

Frame structure:
1. magic bytes,
2. protocol version,
3. message type,
4. sequence number,
5. payload length,
6. payload,
7. CRC16.

## Scope split

- Nano no longer handles display logic.
- Nano no longer handles RS-485/Modbus logic.
- RS-485/Modbus is handled only by ESP32.

## Fail-safe behavior

- ESP32 enforces safe-stop if Nano telemetry is stale/lost.
- Nano forces safe output state when valid ESP32 commands time out.

## ESP32 API

- `GET /` — web UI
- `GET /state` — current state
- `GET /settings` — active settings
- `POST /set?param=...&value=...` — change setting
- `POST /action?pump=well|house&cmd=...` — control actions
- `GET /logs_well`, `GET /logs_house`
- `POST /clear_logs_well`, `POST /clear_logs_house`
