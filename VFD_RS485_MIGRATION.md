# VFD RS-485 / Modbus RTU migration (Nano -> ESP32)

## Что изменено

- RS-485/Modbus RTU для VFD полностью перенесён с Arduino Nano на ESP32.
- Nano (`v1.ino`) выполняет только роль I/O bridge:
  - уровни,
  - аналоговые каналы,
  - давления,
  - управление реле скважины,
  - телеметрия по UART в ответ на команды ESP32.
- ESP32 (`esp32_controller.ino`) теперь:
  - управляет VFD командами RUN/STOP,
  - задаёт частоту,
  - читает ток и статус VFD по Modbus RTU,
  - использует эти значения в существующих защитах.

## Новая проводка MAX485 к ESP32

- `GPIO25` -> `DI` (MAX485 TX)
- `GPIO26` <- `RO` (MAX485 RX), через делитель если `RO=5V`:
  - `RO -> 10k -> GPIO26`
  - `GPIO26 -> 20k -> GND`
- `RE` + `DE` (вместе) -> `GPIO27`
  - `LOW` = receive
  - `HIGH` = transmit

## Изменения Nano <-> ESP32

- Контроллерная связь переведена с I2C на UART.
- Протокол кадров: magic/version/type/seq/payloadLen/payload/CRC16.
- Nano не содержит RS-485/Modbus логики VFD.

## Совместимость

- Обновлять нужно обе прошивки одновременно (`v1.ino` и `esp32_controller.ino`).
