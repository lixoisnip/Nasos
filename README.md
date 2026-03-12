# Nasos: Arduino Nano + ESP32

## Важно: какая прошивка для какой архитектуры

- **`Osnova.ino`** — историческая монолитная версия (референс).
- Для split-архитектуры использовать:
  - **Arduino Nano: `v1.ino`**
  - **ESP32: `esp32_controller.ino`**

## Роли контроллеров

1. **`v1.ino` (Arduino Nano, I/O bridge)**
   - уровни L1-L4;
   - аналоговые входы (токи/давления);
   - релейный выход скважинного насоса;
   - локальная фильтрация датчиков;
   - ответ телеметрией на команды ESP32 по UART.

2. **`esp32_controller.ino` (ESP32, master-логика + VFD Modbus RTU)**
   - основная логика и защиты;
   - web UI / API / настройки / логи;
   - связь с Nano по UART (Serial2);
   - RS-485 / Modbus RTU с VFD.

3. **`data/index.html`**
   - веб-панель мониторинга и настройки.

## Подключение Nano ↔ ESP32 (UART, вместо I2C)

Контроллерная связь Nano↔ESP32 переведена с I2C на UART.

### ESP32 (Serial2)
- `ESP32 GPIO16` = `RX2` (прием от Nano)
- `ESP32 GPIO17` = `TX2` (передача в Nano)

### Arduino Nano (SoftwareSerial)
- `Nano D10` = `TX` (в сторону ESP32 RX2)
- `Nano D11` = `RX` (от ESP32 TX2)
- USB `Serial` можно оставить для отладки.

### Согласование уровней (обязательно)

`Nano TX (5V) -> ESP32 RX (3.3V)` через делитель:

- `Nano TX --10k--+-- ESP32 RX`
- `               |`
- `              20k`
- `               |`
- `              GND`

`ESP32 TX -> Nano RX` можно подключать напрямую.

### Общая земля
- `GND Nano` ↔ `GND ESP32` (**обязательно**)

## Параметры UART link

- Скорость: **38400 baud**
- Кадры: бинарные, с заголовком и CRC16
- Модель обмена: request-response
  - ESP32 отправляет командный кадр;
  - Nano валидирует команду и применяет выходы;
  - Nano отправляет кадр телеметрии в ответ.

## Протокол Nano↔ESP32

Используется короткий бинарный протокол с полями:
- magic bytes,
- protocol version,
- message type,
- sequence,
- payload length,
- payload,
- CRC16.

Смысл инженерных значений и защит сохранен как в предыдущей версии.

## Fail-safe

- На ESP32: при отсутствии свежей валидной телеметрии link считается потерянным, насосы переводятся в безопасный стоп.
- На Nano: при просрочке валидных команд от ESP32 релейный выход снимается (safe output).

## RS-485 / Modbus

RS-485/Modbus для VFD остается на стороне ESP32.
Nano не содержит RS-485/Modbus логики VFD.

## API ESP32

- `GET /` — веб-интерфейс
- `GET /state` — текущее состояние
- `GET /settings` — текущие настройки
- `POST /set?param=...&value=...` — изменить параметр
- `POST /action?pump=well|house&cmd=...` — команды (`reset_alarm`, `force_on`, `force_off`)
- `GET /logs_well`, `GET /logs_house`
- `POST /clear_logs_well`, `POST /clear_logs_house`

## Wi‑Fi

ESP32 работает в режиме **AP + STA**:
- AP: `Nasos-ESP32` / `12345678`
- STA: `wifi_ssid` / `wifi_pass` в настройках

Если роутер недоступен, интерфейс доступен через AP ESP32 `192.168.4.1`.
