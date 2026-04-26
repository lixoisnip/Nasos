# Автоматика на Arduino / Arduino Pump Automation

## RU

Проект автоматизирует работу **скважинного** и **домашнего (через VFD)** насосов.

### Архитектура
- **Arduino Nano (`v1.ino`)**: сбор уровней, токов, давления; управление реле скважинного насоса; I²C‑обмен с ESP32.
- **ESP32 (`esp32_controller.ino` + модули)**: управляющая логика, защиты, Modbus RTU для VFD, веб‑интерфейс.

### Модули ESP32
- `telemetry.{h,cpp}` — опрос Nano и VFD.
- `settings.{h,cpp}` — загрузка/сохранение/валидация настроек и Wi‑Fi в LittleFS (`/settings.json`).
- `control.{h,cpp}` — `runWellLogic`, `runHouseLogic`, защиты, антиспам команд VFD.
- `web_server.{h,cpp}` — Async API, JSON состояния, сохранение настроек.
- `modbus.{h,cpp}` — CRC, `vfdReadReg`/`vfdWriteReg`, таймауты.
- `logging.{h,cpp}` — файловые логи, ротация, очистка/скачивание.

### Подключение (основное)
- ESP32 ↔ Nano (I²C): GPIO21(SDA), GPIO22(SCL), GND, адрес `0x10`.
- ESP32 ↔ MAX485: TX2=GPIO17, RX2=GPIO16, DE/RE=GPIO27.
- MAX485 ↔ VFD: RS‑485 A/B согласно документации VFD.

### Требования
- Arduino IDE 2.x или PlatformIO.
- Для ESP32: библиотеки `ESPAsyncWebServer`, `AsyncTCP`, `ArduinoJson`.
- Файловая система LittleFS.

### Прошивка
1. Прошить Nano скетчем `v1.ino`.
2. Загрузить веб‑файлы из `data/` в LittleFS ESP32.
3. Прошить ESP32 (все `.ino/.h/.cpp` в корне проекта).

### Настройка Wi‑Fi
- Параметры хранятся в `/settings.json`.
- Если файла нет, создаются дефолты и поднимается AP `Nasos-ESP32`.
- Изменение Wi‑Fi доступно через `/settings` в веб‑панели.

### API (кратко)
- `GET /state`
- `GET /settings`, `POST /settings`
- `POST /pump` (`unit`, `action`)
- `GET /logs_well`, `GET /logs_house`
- `POST /clear_logs_well`, `POST /clear_logs_house`
- `GET /download_logs_well`, `GET /download_logs_house`

---

## EN

Automation project for **well pump** and **house pump via VFD**.

### Components
- **Arduino Nano**: sensor sampling + relay output + I²C slave.
- **ESP32**: control/protection logic, VFD Modbus RTU, async web UI/API.

### Build/Flash
1. Flash `v1.ino` to Nano.
2. Upload `data/` into ESP32 LittleFS.
3. Flash ESP32 firmware from root sources.

### Wi‑Fi
Wi‑Fi credentials are stored in LittleFS (`/settings.json`) and editable via web API/UI.

