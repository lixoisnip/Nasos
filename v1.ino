#ifndef NASOS_V1_INO_GUARD
#define NASOS_V1_INO_GUARD

// =====================================================
// СКВАЖИННЫЙ НАСОС + ДОМАШНИЙ НАСОС v5.3
// Добавлена защита по сухому ходу для домашнего насоса
// =====================================================

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <EEPROM.h>

// Пины
#define TFT_CS        10
#define TFT_DC         9
#define TFT_RST        8
#define RELAY_WELL     3
#define PIN_RS485_DE_RE 7
#define ACS_PIN       A1    // Ток скважинного насоса
#define PIN_CURRENT   A0    // Ток домашнего насоса
#define PRESSURE_PIN  A2    // Давление скважинного насоса
#define PIN_PRESSURE  A3    // Давление домашнего насоса
#define L1            A4
#define L2            A5
#define L3            A6
#define L4            A7

Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);

// Цвета
#define BLACK   ST77XX_BLACK
#define WHITE   ST77XX_WHITE
#define GREEN   ST77XX_GREEN
#define RED     ST77XX_RED
#define YELLOW  ST77XX_YELLOW
#define GRAY    0x7BEF
#define CYAN    ST77XX_CYAN

// Зоны дисплея
#define Z1 40
#define Z2 60
#define Z3 80
#define Z4 80
#define Z5 20
#define Y1 0
#define Y2 (Y1+Z1)
#define Y3 (Y2+Z2)
#define Y4 (Y3+Z3)
#define Y5 (Y4+Z4)

// Уставки скважинного насоса
namespace cfg {
  constexpr float TARGET_MIN      = 5.0f;
  constexpr float L_PER_MIN       = 30.0f;
  constexpr float MIN_PAUSE       = 10.0f;
  constexpr float MAX_PAUSE       = 90.0f;

  constexpr float CURRENT_DRY          = 3.3f;
  constexpr float CURRENT_OVERLOAD     = 4.3f;
  constexpr float CURRENT_EMERGENCY    = 6.0f;
  constexpr float CURRENT_MIN_START    = 2.9f;  // Изменено с 2.5А на 2.9А

  constexpr float PRESSURE_MIN_OK      = 0.30f;
  constexpr float PRESSURE_WARNING     = 1.20f;
  constexpr float PRESSURE_BLOCK       = 1.50f;

  constexpr unsigned long OVERLOAD_DELAY_MS = 5000UL;
  constexpr unsigned long DRY_DELAY_MS      = 8000UL;
  constexpr unsigned long PRESSURE_CHECK_DELAY = 8000UL;
  constexpr int MAX_FAILED_STARTS = 3;
}

// Уставки домашнего насоса
namespace cfgHouse {
  constexpr float SETPOINT_BAR     = 1.00f;
  constexpr float HYST_ON          = 0.50f;
  constexpr float HYST_OFF         = 1.18f;
  constexpr float MIN_FREQ         = 28.0f;
  constexpr float MAX_FREQ         = 50.0f;
  constexpr float SHUTDOWN_FREQ    = 34.0f;
  
  // Защиты по току
  constexpr float CURRENT_NORMAL_MIN  = 0.6f;
  constexpr float CURRENT_NORMAL_MAX  = 1.0f;
  constexpr float CURRENT_DRY         = 0.4f;
  constexpr float CURRENT_OVERLOAD    = 1.3f;
  constexpr float CURRENT_EMERGENCY   = 1.5f;
  
  constexpr unsigned long OVERLOAD_DELAY_MS = 5000UL;
  constexpr unsigned long DRY_DELAY_MS = 8000UL;
  constexpr unsigned long START_CURRENT_IGNORE_MS = 2000UL;
  
  // Защита по сухому ходу по давлению
  constexpr unsigned long DRY_PRESSURE_START_TIMEOUT = 10000UL;  // 10 секунд для старта
  constexpr unsigned long DRY_PRESSURE_WORK_TIMEOUT = 15000UL;   // 15 секунд для работы
  constexpr float PRESSURE_RISE_THRESHOLD = 0.1f;                // Порог начала роста давления
  
  // ПИД параметры
  constexpr float Kp = 260.0f;
  constexpr float Ki = 0.45f;
  constexpr float INTEGRAL_LIMIT = 18.0f;
  
  // Тайминги
  constexpr unsigned long PID_PERIOD = 300UL;
  constexpr unsigned long FREQ_STEP_DELAY = 500UL;
}

// EEPROM
namespace ee {
  constexpr int ADDR_MAGIC      = 0;
  constexpr int ADDR_INTENTION  = 4;
  constexpr int ADDR_TOTAL_L    = 8;
  constexpr int ADDR_FAILED_CNT = 12;
  constexpr int ADDR_PAUSE_MS   = 16;
  constexpr uint32_t MAGIC = 0xBEEFCAFE;
}

// Намерение насоса
enum class PumpIntention : uint8_t {
  UNKNOWN,
  TARGET_REACHED,
  PUMPING_TO_L4
};

// Глобальные
float currentZeroOffset = 512.0f;
constexpr unsigned long LEVEL_FILTER_MS = 2000UL;
constexpr unsigned long INIT_DELAY_MS   = 6000UL;

struct LevelFilter {
  bool current = false; bool raw = false; unsigned long changeTime = 0;
} levelFilter[4];

// Структура состояния скважинного насоса
struct State {
  enum class Mode { INIT, WAIT, STARTING, RUN, FAIL } mode = Mode::INIT;

  bool blocked = false;
  bool alarm = false;
  bool filterWarning = false;
  bool pressureBlock = false;

  float pauseMs = 0;
  unsigned long pauseStart = 0;
  unsigned long workStart = 0;
  unsigned long initStart = 0;
  unsigned long pressureCheckStart = 0;

  int failedStartCount = 0;

  float totalLiters = 0;
  float savedLiters = 0;

  float current = 0;
  float pressureBar = 0.0f;

  unsigned long lastWorkSec = 0;
  bool lastWorkGood = false;

  bool level[4] = {false};
  bool needPump = false;
  bool targetOk = false;

  PumpIntention intention = PumpIntention::UNKNOWN;

  unsigned long lastUpd = 0;
  unsigned long flash = 0;
  bool flashOn = false;

  float pidInt = 0;
  float pidLastE = 0;
} st;

// Структура состояния домашнего насоса
struct HouseState {
  enum class Mode { 
    WAIT_WATER,      // Ожидание воды (L1 и L2)
    READY,           // Готов к работе (вода есть, давление в норме)
    RUNNING,         // Работает
    STOPPED          // Остановлен (нет L1)
  } mode = Mode::WAIT_WATER;
  
  bool running = false;
  bool blocked = false;
  bool alarm = false;
  
  float current = 0.0f;
  float pressureBar = 0.0f;
  float targetFreq = cfgHouse::MIN_FREQ;
  float actualFreq = cfgHouse::MIN_FREQ;
  
  unsigned long lastPID = 0;
  unsigned long lastFreqChange = 0;
  unsigned long overloadStart = 0;
  unsigned long dryStart = 0;
  unsigned long startCurrentIgnoreUntil = 0;
  
  // Защита по сухому ходу по давлению
  unsigned long dryPressureStart = 0;        // Таймер сухого хода по давлению
  bool pressureStartedToRise = false;        // Флаг, что давление начало расти
  float initialPressure = 0.0f;              // Давление на момент старта
  
  float integral = 0.0f;
  
  // Фильтр давления
  const int SMOOTHING = 6;
  float pressureBuffer[6];
  int bufIdx = 0;
} hs;

// =====================================================
// ФУНКЦИИ ДОМАШНЕГО НАСОСА
// =====================================================

// Modbus RTU функции
void txMode() { 
  digitalWrite(PIN_RS485_DE_RE, HIGH); 
  delayMicroseconds(100);
}

void rxMode() { 
  delayMicroseconds(100); 
  digitalWrite(PIN_RS485_DE_RE, LOW);
}

// Расчет CRC-16 Modbus
uint16_t calculateCRC(uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc = crc >> 1;
      }
    }
  }
  return crc;
}

// Запись регистра Modbus
void writeReg(uint16_t addr, uint16_t val) {
  uint8_t frame[8];
  
  // Адрес устройства (08 из документа)
  frame[0] = 0x08;
  // Код функции 06 (запись одиночного регистра)
  frame[1] = 0x06;
  // Адрес регистра (старший и младший байты)
  frame[2] = highByte(addr);
  frame[3] = lowByte(addr);
  // Значение (старший и младший байты)
  frame[4] = highByte(val);
  frame[5] = lowByte(val);
  
  // Расчет CRC для 6 байт
  uint16_t crc = calculateCRC(frame, 6);
  
  // Добавляем CRC (младший байт первый!)
  frame[6] = lowByte(crc);
  frame[7] = highByte(crc);
  
  // Отправка
  txMode();
  Serial.write(frame, 8);
  Serial.flush();
  rxMode();
  
  // Пауза для обработки команды
  delay(50);
}

// Установка частоты
void setFrequency(float hz) {
  // Преобразуем Гц в значение регистра (0.01 Гц единица)
  uint16_t freqValue = (uint16_t)round(hz * 100.0f);
  writeReg(0x9CA6, freqValue);
}

// Команда запуска VFD
void vfdStart() { 
  // Команда START: 08 06 9C A7 00 01 D7 20
  writeReg(0x9CA7, 0x0001);
  hs.running = true;
  
  Serial.println("VFD: START command sent");
}

// Команда остановки VFD
void vfdStop() { 
  // Команда STOP: 08 06 9C A7 00 00 16 E0
  writeReg(0x9CA7, 0x0000);
  hs.running = false;
  
  Serial.println("VFD: STOP command sent");
}

// Инициализация VFD
void initVFD() {
  // 1. Установка управления по RS-485 (не с кнопки)
  writeReg(0x9C41, 0x0002);
  delay(200);
  
  // 2. Установка источника задания частоты - RS-485
  writeReg(0x9C40, 0x0005);
  delay(200);
  
  // 3. Установка начальной частоты 0 Гц
  writeReg(0x9CA6, 0x0000);
  delay(200);
  
  // 4. Останов насоса (на всякий случай)
  vfdStop();
  delay(200);
  
  Serial.println("VFD: Initialization complete");
}

// Калибровка давления домашнего насоса
float calibratePressure(float rawBar) {
  float corrected = rawBar - 0.152f;
  corrected = corrected * (1.80f / 1.95f);
  return max(0.0f, corrected);
}

// Чтение датчиков домашнего насоса
void readHouseSensors() {
  // Давление
  int raw = analogRead(PIN_PRESSURE);
  float voltage = raw * 5.0f / 1023.0f;
  float rawPressure = max(0.0f, (voltage - 0.5f) * 12.0f / 4.0f);
  float calibrated = calibratePressure(rawPressure);
  
  hs.pressureBuffer[hs.bufIdx] = calibrated;
  hs.bufIdx = (hs.bufIdx + 1) % hs.SMOOTHING;
  
  float sum = 0.0f;
  for (int i = 0; i < hs.SMOOTHING; i++) sum += hs.pressureBuffer[i];
  hs.pressureBar = sum / hs.SMOOTHING;
  
  // Ток двигателя
  raw = analogRead(PIN_CURRENT);
  voltage = raw * 5.0f / 1023.0f;
  hs.current = voltage * 2.0f;
  if (hs.current < 0.1f) hs.current = 0.0f;
}

// Защиты домашнего насоса по току
void applyHouseProtections(unsigned long now) {
  bool ignoreCurrentProtection = hs.running && (now < hs.startCurrentIgnoreUntil);

  if (ignoreCurrentProtection) {
    hs.overloadStart = 0;
    hs.dryStart = 0;
    return;
  }

  // Аварийная перегрузка (1.5А)
  if (hs.current >= cfgHouse::CURRENT_EMERGENCY) {
    hs.blocked = hs.alarm = true;
    vfdStop();
    hs.mode = HouseState::Mode::STOPPED;
    Serial.println("HOUSE: Emergency overload!");
    return;
  }
  
  // Перегрузка 1.3А в течение 5 секунд
  if (hs.current >= cfgHouse::CURRENT_OVERLOAD) {
    if (hs.overloadStart == 0) hs.overloadStart = now;
    else if (now - hs.overloadStart >= cfgHouse::OVERLOAD_DELAY_MS) {
      hs.blocked = hs.alarm = true;
      vfdStop();
      hs.mode = HouseState::Mode::STOPPED;
      Serial.println("HOUSE: Overload protection!");
    }
  } else {
    hs.overloadStart = 0;
  }
  
  // Сухой ход по току (менее 0.4А при работающем насосе)
  if (hs.running && hs.current < cfgHouse::CURRENT_DRY) {
    if (hs.dryStart == 0) hs.dryStart = now;
    else if (now - hs.dryStart >= cfgHouse::DRY_DELAY_MS) {
      hs.blocked = hs.alarm = true;
      vfdStop();
      hs.mode = HouseState::Mode::STOPPED;
      Serial.println("HOUSE: Dry run protection (current)!");
    }
  } else {
    hs.dryStart = 0;
  }
}

// Защита по сухому ходу по давлению
void checkDryPressureProtection(unsigned long now) {
  if (!hs.running) {
    // Насос не работает - сбрасываем таймеры
    hs.dryPressureStart = 0;
    hs.pressureStartedToRise = false;
    return;
  }
  
  // 1. ЗАЩИТА ПРИ СТАРТЕ: давление должно начать расти в течение 10 секунд
  if (!hs.pressureStartedToRise) {
    // Проверяем, превысило ли давление порог начала роста
    if (hs.pressureBar > hs.initialPressure + cfgHouse::PRESSURE_RISE_THRESHOLD) {
      // Давление начало расти - снимаем защиту старта
      hs.pressureStartedToRise = true;
      hs.dryPressureStart = 0; // Сбрасываем таймер
      Serial.println("HOUSE: Pressure started to rise, start protection OK");
    } 
    // Если давление не начало расти и прошло более 10 секунд - АВАРИЯ
    else if (hs.dryPressureStart != 0 && 
             (now - hs.dryPressureStart > cfgHouse::DRY_PRESSURE_START_TIMEOUT)) {
      hs.blocked = hs.alarm = true;
      vfdStop();
      hs.running = false;
      hs.mode = HouseState::Mode::STOPPED;
      Serial.println("HOUSE: DRY RUN - No pressure rise in 10s!");
      return;
    }
  } 
  // 2. ЗАЩИТА ВО ВРЕМЯ РАБОТЫ: если давление упало ниже 0.5 бар и не восстанавливается
  else if (hs.pressureBar <= cfgHouse::HYST_ON) {
    // Запускаем таймер низкого давления
    if (hs.dryPressureStart == 0) {
      hs.dryPressureStart = now;
      Serial.println("HOUSE: Low pressure detected, starting 15s timer");
    }
    // Если низкое давление держится более 15 секунд - АВАРИЯ
    else if (now - hs.dryPressureStart > cfgHouse::DRY_PRESSURE_WORK_TIMEOUT) {
      hs.blocked = hs.alarm = true;
      vfdStop();
      hs.running = false;
      hs.mode = HouseState::Mode::STOPPED;
      Serial.println("HOUSE: DRY RUN - Low pressure for 15s!");
      return;
    }
  } 
  else {
    // Давление в норме (>0.5 бар) - сбрасываем таймер
    if (hs.dryPressureStart != 0) {
      hs.dryPressureStart = 0;
      Serial.println("HOUSE: Pressure restored, timer reset");
    }
  }
}

// Обработка работающего насоса
void handleRunningState(unsigned long now) {
  // Сначала проверяем защиту по сухому ходу по давлению
  checkDryPressureProtection(now);
  if (hs.blocked) return;
  
  // Остановка по достижению рабочего давления
  if (hs.pressureBar >= cfgHouse::HYST_OFF) {
    vfdStop();
    hs.running = false;
    hs.mode = HouseState::Mode::READY;
    hs.integral = 0.0f;
    hs.targetFreq = hs.actualFreq = cfgHouse::MIN_FREQ;
    
    // Сбрасываем таймеры защиты по давлению
    hs.dryPressureStart = 0;
    hs.pressureStartedToRise = false;
    
    Serial.println("HOUSE: Stopped by pressure (1.18 bar)");
    return;
  }
  
  // ПИД регулирование
  if (now - hs.lastPID >= cfgHouse::PID_PERIOD) {
    hs.lastPID = now;
    
    float error = cfgHouse::SETPOINT_BAR - hs.pressureBar;
    
    hs.integral += error * (cfgHouse::PID_PERIOD / 1000.0f);
    hs.integral = constrain(hs.integral, 
                           -cfgHouse::INTEGRAL_LIMIT, 
                           cfgHouse::INTEGRAL_LIMIT);
    
    hs.targetFreq = cfgHouse::MIN_FREQ + cfgHouse::Kp * error + cfgHouse::Ki * hs.integral;
    hs.targetFreq = constrain(hs.targetFreq, 
                             cfgHouse::MIN_FREQ, 
                             cfgHouse::MAX_FREQ);
  }
  
  // Плавное изменение частоты
  if (now - hs.lastFreqChange >= cfgHouse::FREQ_STEP_DELAY) {
    hs.lastFreqChange = now;
    
    float step = (abs(hs.targetFreq - hs.actualFreq) > 6.0f) ? 2.0f : 1.0f;
    
    if (hs.actualFreq < hs.targetFreq) {
      hs.actualFreq = min(hs.actualFreq + step, hs.targetFreq);
    } else if (hs.actualFreq > hs.targetFreq) {
      hs.actualFreq = max(hs.actualFreq - step, hs.targetFreq);
    }
    
    setFrequency(hs.actualFreq);
  }
}

// Управление домашним насосом
void runHousePump(unsigned long now) {
  if (hs.blocked) return;
  
  bool L1 = st.level[0];
  bool L2 = st.level[1];
  
  // Определяем режим
  if (!L1) {
    // Нет воды вообще - ОСТАНОВ
    hs.mode = HouseState::Mode::STOPPED;
    if (hs.running) {
      vfdStop();
      hs.running = false;
      hs.integral = 0.0f;
      hs.targetFreq = hs.actualFreq = cfgHouse::MIN_FREQ;
      
      // Сбрасываем таймеры защиты по давлению
      hs.dryPressureStart = 0;
      hs.pressureStartedToRise = false;
      
      Serial.println("HOUSE: No L1, stopped");
    }
    return;
  }
  
  if (!L2) {
    // Есть только L1 - можно работать, но режим WAIT_WATER
    hs.mode = HouseState::Mode::WAIT_WATER;
    // Если насос уже работает - НЕ останавливаем его
    if (hs.running) {
      hs.mode = HouseState::Mode::RUNNING;
      handleRunningState(now);
    } else {
      Serial.println("HOUSE: Waiting for L2");
    }
    // Если не работает - не запускаем, ждем L2
    return;
  }
  
  // Есть и L1 и L2
  if (!hs.running) {
    if (hs.pressureBar <= cfgHouse::HYST_ON) {
      // Запускаем насос на 50 Гц
      hs.actualFreq = cfgHouse::MAX_FREQ;
      hs.targetFreq = cfgHouse::MAX_FREQ;
      setFrequency(hs.actualFreq);
      delay(100);
      
      vfdStart();
      hs.running = true;
      hs.mode = HouseState::Mode::RUNNING;
      hs.integral = 0.0f;
      hs.lastFreqChange = now;
      hs.startCurrentIgnoreUntil = now + cfgHouse::START_CURRENT_IGNORE_MS;
      
      // Инициализируем защиту по давлению при старте
      hs.initialPressure = hs.pressureBar;
      hs.dryPressureStart = now;
      hs.pressureStartedToRise = false;
      
      Serial.println("HOUSE: Started pump, pressure protection active");
    } else {
      // Давление в норме, ждем
      hs.mode = HouseState::Mode::READY;
    }
  } else {
    // Насос работает
    hs.mode = HouseState::Mode::RUNNING;
    handleRunningState(now);
  }
}

// =====================================================
// ФУНКЦИИ СКВАЖИННОГО НАСОСА
// =====================================================

float readCurrent() {
  static float smooth = 0.0f;
  long sumSq = 0;
  for (int i = 0; i < 120; i++) {
    int diff = analogRead(ACS_PIN) - (int)currentZeroOffset;
    sumSq += (long)diff * diff;
  }
  float current = (sqrt(sumSq / 120.0f) * 5.0f / 1023.0f) / 0.066f;
  if (current < 0.25f) current = 0.0f;
  smooth = 0.12f * current + 0.88f * smooth;
  return smooth;
}

float readPressureBar() {
  static float filtered = 0.0f;
  int raw = analogRead(PRESSURE_PIN);
  float voltage = raw * (5.0f / 1023.0f);
  if (voltage < 0.5f) voltage = 0.5f;
  float measured = (voltage - 0.5f) * 12.0f;
  float realBar = measured / 5.2f;
  filtered = filtered * 0.9f + realBar * 0.1f;
  return constrain(filtered, 0.0f, 15.0f);
}

void readAndFilterLevels(unsigned long now) {
  constexpr int THRESH = 700;
  bool raw[4] = {
    analogRead(L1) > THRESH, analogRead(L2) > THRESH,
    analogRead(L3) > THRESH, analogRead(L4) > THRESH
  };
  for (int i = 0; i < 4; i++) {
    if (raw[i] != levelFilter[i].raw) {
      levelFilter[i].raw = raw[i];
      levelFilter[i].changeTime = now;
    }
    if (now - levelFilter[i].changeTime >= LEVEL_FILTER_MS) {
      levelFilter[i].current = levelFilter[i].raw;
    }
    st.level[i] = levelFilter[i].current;
  }
}

// Обновление необходимости работы скважинного насоса
void updateWellPumpNeed() {
  bool L2 = st.level[1];
  bool L4 = st.level[3];

  // Основная логика: если нет L2 - нужно качать воду
  if (!L2) {
    st.needPump = true;
    st.targetOk = false;
    st.intention = PumpIntention::PUMPING_TO_L4;
  }
  // Если есть L2 и L4 - бак полный, не нужно качать
  else if (L2 && L4) {
    st.needPump = false;
    st.targetOk = true;
    st.intention = PumpIntention::TARGET_REACHED;
  }
  // Если есть L2, но нет L4 - смотрим на предыдущее намерение
  else if (L2 && !L4) {
    if (st.intention == PumpIntention::PUMPING_TO_L4) {
      // Ранее качали до L4 - продолжаем качать
      st.needPump = true;
      st.targetOk = false;
    } else {
      // Был полный бак - не нужно качать
      st.needPump = false;
      st.targetOk = true;
    }
  }
}

void runMachine(unsigned long now) {
  if (st.blocked || st.pressureBlock) return;

  switch (st.mode) {
    case State::Mode::WAIT:
      // Запускаем если нужно качать И прошло время паузы
      if (st.needPump && (st.pauseMs == 0 || now - st.pauseStart >= st.pauseMs)) {
        digitalWrite(RELAY_WELL, LOW);
        st.mode = State::Mode::STARTING;
        st.pauseStart = now;
        st.pressureCheckStart = now;
        st.intention = PumpIntention::PUMPING_TO_L4;
        saveState();
        Serial.println("WELL: Starting pump (L2 empty)");
      }
      break;

    case State::Mode::STARTING:
      if (now - st.pauseStart >= 15000UL) {
        if (st.current >= cfg::CURRENT_MIN_START) {  // Теперь 2.9А вместо 2.5А
          if (now - st.pressureCheckStart >= cfg::PRESSURE_CHECK_DELAY) {
            if (st.pressureBar >= cfg::PRESSURE_MIN_OK) {
              st.workStart = now;
              st.failedStartCount = 0;
              st.mode = State::Mode::RUN;
              Serial.println("WELL: Pump running");
            } else {
              digitalWrite(RELAY_WELL, HIGH);
              st.failedStartCount++;
              if (st.failedStartCount >= cfg::MAX_FAILED_STARTS) {
                st.blocked = st.alarm = true;
                st.mode = State::Mode::FAIL;
                saveState();
                Serial.println("WELL: Blocked - failed starts");
              } else {
                st.mode = State::Mode::WAIT;
                st.pauseStart = now;
                Serial.println("WELL: No pressure, waiting");
              }
            }
          }
        } else {
          digitalWrite(RELAY_WELL, HIGH);
          st.failedStartCount++;
          if (st.failedStartCount >= cfg::MAX_FAILED_STARTS) {
            st.blocked = st.alarm = true;
            st.mode = State::Mode::FAIL;
            saveState();
            Serial.println("WELL: Blocked - failed starts");
          } else {
            st.mode = State::Mode::WAIT;
            st.pauseStart = now;
            if (st.pauseMs == 0) st.pauseMs = cfg::MIN_PAUSE * 60000;
            Serial.println("WELL: Current too low, waiting");
          }
        }
      }
      break;

    case State::Mode::RUN:
      // Останов при достижении L4
      if (st.level[1] && st.level[3]) {
        digitalWrite(RELAY_WELL, HIGH);
        float workedMin = (now - st.workStart) / 60000.0f;
        st.totalLiters += workedMin * cfg::L_PER_MIN;
        st.lastWorkSec = (now - st.workStart) / 1000;
        st.lastWorkGood = (workedMin >= cfg::TARGET_MIN);

        st.intention = PumpIntention::TARGET_REACHED;
        saveState();
        st.pauseStart = now;
        st.mode = State::Mode::WAIT;
        Serial.println("WELL: Stopped (L4 reached)");
      }
      // Останов при сухом ходе
      else if (st.current < cfg::CURRENT_DRY) {
        digitalWrite(RELAY_WELL, HIGH);
        float workedMin = (now - st.workStart) / 60000.0f;
        st.totalLiters += workedMin * cfg::L_PER_MIN;
        st.lastWorkSec = (now - st.workStart) / 1000;

        st.intention = PumpIntention::PUMPING_TO_L4;
        adjustPID(workedMin);
        saveState();
        st.pauseStart = now;
        st.mode = State::Mode::WAIT;
        Serial.println("WELL: Stopped (dry run)");
      }
      break;
  }
}

void applyProtections(unsigned long now) {
  static unsigned long dryStart = 0, overloadStart = 0;

  if (st.current > cfg::CURRENT_EMERGENCY) {
    st.blocked = st.alarm = true;
    digitalWrite(RELAY_WELL, HIGH);
    st.mode = State::Mode::FAIL;
    saveState();
    return;
  }

  if (st.current > cfg::CURRENT_OVERLOAD) {
    if (overloadStart == 0) overloadStart = now;
    else if (now - overloadStart >= cfg::OVERLOAD_DELAY_MS) {
      st.blocked = st.alarm = true;
      digitalWrite(RELAY_WELL, HIGH);
      st.mode = State::Mode::FAIL;
      saveState();
    }
  } else overloadStart = 0;

  if (st.current < cfg::CURRENT_DRY && st.mode == State::Mode::RUN) {
    if (dryStart == 0) dryStart = now;
    else if (now - dryStart >= cfg::DRY_DELAY_MS) {
      st.blocked = st.alarm = true;
      digitalWrite(RELAY_WELL, HIGH);
      st.mode = State::Mode::FAIL;
      saveState();
    }
  } else dryStart = 0;

  if (st.pressureBar >= cfg::PRESSURE_BLOCK) {
    st.pressureBlock = st.alarm = true;
    digitalWrite(RELAY_WELL, HIGH);
    st.mode = State::Mode::FAIL;
    saveState();
  } else if (st.pressureBar >= cfg::PRESSURE_WARNING) {
    st.filterWarning = true;
  } else {
    st.filterWarning = false;
  }
}

void adjustPID(float m) {
  float e = cfg::TARGET_MIN - m;
  if (abs(e) > 2.0f) st.pidInt += e * 2.5f; else st.pidInt += e;
  st.pidInt = constrain(st.pidInt, -40, 40);
  float p = 1.5f * e * (abs(e) > 2.0f ? 2.5f : 1.0f);
  float i = 0.1f * st.pidInt * (abs(e) > 2.0f ? 2.5f : 1.0f);
  float d = 0.2f * (e - st.pidLastE);
  st.pidLastE = e;
  float cur = st.pauseMs / 60000.0f;
  cur = constrain(cur + p + i + d, cfg::MIN_PAUSE, cfg::MAX_PAUSE);
  st.pauseMs = cur * 60000.0f;
}

// =====================================================
// ФУНКЦИИ ОТОБРАЖЕНИЯ
// =====================================================

void drawStatic() {
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(5, 10);   tft.print("S:");
  tft.setCursor(70, 10);  tft.print("A:");
  tft.setCursor(135, 10); tft.print("W:");

  tft.setTextSize(1);
  for (int i = 0; i < 4; i++) {
    int x = 30 + i * 45;
    tft.drawRoundRect(x, Y2 + 15, 30, 30, 5, GRAY);
    tft.setCursor(x + 11, Y2 + 25); tft.print(i + 1);
  }

  tft.setCursor(5, Y3 + 10);   tft.print("I1:");
  tft.setCursor(5, Y3 + 30);   tft.print("T1:");
  tft.setCursor(5, Y3 + 50);   tft.print("T2:");
  tft.setCursor(120, Y3 + 10); tft.print("V1:");
  tft.setCursor(120, Y3 + 30); tft.print("P1:");

  tft.setCursor(5, Y4 + 10);   tft.print("I2:");
  tft.setCursor(5, Y4 + 30);   tft.print("H2:");
  tft.setCursor(5, Y4 + 50);   tft.print("P2:");
  tft.setCursor(120, Y4 + 10); tft.print("ST2:");

  tft.setCursor(5, Y5 + 5);    tft.print("FIL:");
  tft.setCursor(120, Y5 + 5);  tft.print("PRO:");
}

void updateDisplay(unsigned long now) {
  // Обновление скважинного насоса
  tft.fillRect(25, 10, 35, 16, BLACK); tft.setTextSize(2); tft.setTextColor(GREEN); tft.setCursor(25, 10); tft.print("ON");
  tft.fillRect(90, 10, 35, 16, BLACK); tft.setTextColor(st.alarm ? RED : GREEN); tft.setCursor(90, 10); tft.print(st.alarm ? "Y" : "N");
  tft.fillRect(155, 10, 35, 16, BLACK); tft.setTextColor(st.filterWarning ? YELLOW : GREEN); tft.setCursor(155, 10); tft.print(st.filterWarning ? "Y" : "N");

  for (int i = 0; i < 4; i++) {
    int x = 30 + i * 45;
    tft.fillRoundRect(x, Y2 + 15, 30, 30, 5, st.level[i] ? GREEN : GRAY);
    tft.setTextColor(BLACK); tft.setCursor(x + 11, Y2 + 25); tft.print(i + 1);
  }

  tft.setTextSize(2);
  tft.fillRect(25, Y3 + 10, 90, 16, BLACK);
  tft.setTextColor(st.current > cfg::CURRENT_OVERLOAD ? RED : st.current < cfg::CURRENT_DRY ? YELLOW : WHITE);
  tft.setCursor(25, Y3 + 10); tft.print(st.current, 1); tft.print("A");

  unsigned long sec = (st.mode == State::Mode::RUN) ? (now - st.workStart) / 1000 : st.lastWorkSec;
  tft.fillRect(25, Y3 + 30, 90, 16, BLACK);
  tft.setTextColor(st.mode == State::Mode::RUN ? WHITE : st.lastWorkGood ? GREEN : YELLOW);
  tft.setCursor(25, Y3 + 30);
  tft.print(sec / 60); tft.print(":"); if ((sec % 60) < 10) tft.print("0"); tft.print(sec % 60); tft.print("s");

  tft.fillRect(25, Y3 + 50, 90, 16, BLACK);
  tft.setTextColor(WHITE);
  tft.setCursor(25, Y3 + 50);
  if (st.mode == State::Mode::WAIT) {
    unsigned long left = (st.pauseMs > (now - st.pauseStart)) ? (st.pauseMs - (now - st.pauseStart)) / 1000 : 0;
    uint16_t m = left / 60; uint8_t s = left % 60;
    tft.print(m); tft.print(":"); if (s < 10) tft.print("0"); tft.print(s); tft.print("m");
  } else if (st.mode == State::Mode::INIT) {
    unsigned long left = (INIT_DELAY_MS - (now - st.initStart)) / 1000;
    tft.print("0:"); if (left < 10) tft.print("0"); tft.print(left); tft.print("s");
  } else {
    tft.print((int)(st.pauseMs / 60000)); tft.print("m");
  }

  float vol = st.totalLiters / 1000.0f;
  if (st.mode == State::Mode::RUN) vol += ((now - st.workStart) / 60000.0f * cfg::L_PER_MIN) / 1000.0f;
  tft.fillRect(140, Y3 + 10, 80, 16, BLACK);
  tft.setCursor(140, Y3 + 10); tft.print(vol, 1); tft.print("k");

  tft.fillRect(140, Y3 + 30, 80, 16, BLACK);
  tft.setTextColor(st.filterWarning ? YELLOW : st.pressureBlock ? RED : WHITE);
  tft.setCursor(140, Y3 + 30); tft.print(st.pressureBar, 2); tft.print("b");

  // Обновление домашнего насоса
  tft.setTextSize(2);
  
  // I2: Ток домашнего насоса
  tft.fillRect(25, Y4 + 10, 70, 16, BLACK);
  tft.setCursor(25, Y4 + 10);
  
  if (hs.current >= cfgHouse::CURRENT_EMERGENCY) {
    tft.setTextColor(RED);
  } else if (hs.current >= cfgHouse::CURRENT_OVERLOAD) {
    tft.setTextColor(YELLOW);
  } else if (hs.current < cfgHouse::CURRENT_DRY) {
    tft.setTextColor(RED);
  } else if (hs.current >= cfgHouse::CURRENT_NORMAL_MIN && 
             hs.current <= cfgHouse::CURRENT_NORMAL_MAX) {
    tft.setTextColor(GREEN);
  } else {
    tft.setTextColor(WHITE);
  }
  
  tft.print(hs.current, 1); tft.print("A");
  
  // H2: Частота домашнего насоса
  tft.fillRect(25, Y4 + 30, 70, 16, BLACK);
  tft.setCursor(25, Y4 + 30);
  tft.setTextColor(WHITE);
  tft.print(hs.actualFreq, 1); tft.print("H");
  
  // P2: Давление домашнего насоса
  tft.fillRect(25, Y4 + 50, 70, 16, BLACK);
  tft.setCursor(25, Y4 + 50);
  if (hs.pressureBar >= cfgHouse::HYST_OFF) {
    tft.setTextColor(RED);
  } else {
    tft.setTextColor(WHITE);
  }
  tft.print(hs.pressureBar, 1); tft.print("b");
  
  // ST2: Статус домашнего насоса
  tft.fillRect(140, Y4 + 10, 70, 16, BLACK);
  tft.setCursor(140, Y4 + 10);
  
  if (hs.blocked) {
    tft.setTextColor(RED);
    tft.print("BLOCK");
  } else {
    switch (hs.mode) {
      case HouseState::Mode::WAIT_WATER:
        tft.setTextColor(YELLOW);
        tft.print("WAIT");
        break;
      case HouseState::Mode::READY:
        tft.setTextColor(GREEN);
        tft.print("READY");
        break;
      case HouseState::Mode::RUNNING:
        tft.setTextColor(GREEN);
        tft.print("RUN");
        break;
      case HouseState::Mode::STOPPED:
        tft.setTextColor(RED);
        tft.print("STOP");
        break;
    }
  }

  // Нижняя часть
  tft.setTextSize(1);
  tft.fillRect(30, Y5 + 5, 60, 10, BLACK);
  tft.setTextColor(st.filterWarning ? YELLOW : GREEN);
  tft.setCursor(30, Y5 + 5); tft.print(st.filterWarning ? "BAD" : "OK");

  tft.fillRect(150, Y5 + 5, 60, 10, BLACK);
  tft.setTextColor(st.blocked || st.pressureBlock || hs.blocked ? RED : GREEN);
  tft.setCursor(150, Y5 + 5); tft.print(st.blocked || st.pressureBlock || hs.blocked ? "OFF" : "ON");
}

void handleFlashing(unsigned long now) {
  if (!st.alarm && !st.filterWarning && !st.pressureBlock && !hs.alarm) return;
  if (now - st.flash < (st.pressureBlock ? 200 : 500)) return;
  st.flash = now;
  st.flashOn = !st.flashOn;
  uint16_t color = st.pressureBlock ? RED : st.filterWarning || hs.alarm ? YELLOW : RED;
  tft.fillRect(0, 0, 240, Z1, st.flashOn ? color : BLACK);
  tft.setTextSize(2);
  tft.setTextColor(st.flashOn ? BLACK : WHITE);
  tft.setCursor(5, 10); tft.print("S:");
  tft.setCursor(70, 10); tft.print("A:");
  tft.setCursor(135, 10); tft.print("W:");
}

void saveState() {
  EEPROM.put(ee::ADDR_MAGIC, ee::MAGIC);
  EEPROM.put(ee::ADDR_INTENTION, (uint8_t)st.intention);
  EEPROM.put(ee::ADDR_TOTAL_L, st.totalLiters);
  EEPROM.put(ee::ADDR_FAILED_CNT, (int16_t)st.failedStartCount);
  EEPROM.put(ee::ADDR_PAUSE_MS, st.pauseMs);
}

void loadState() {
  uint32_t magic;
  EEPROM.get(ee::ADDR_MAGIC, magic);
  if (magic != ee::MAGIC) return;

  uint8_t i; EEPROM.get(ee::ADDR_INTENTION, i); st.intention = (PumpIntention)i;
  EEPROM.get(ee::ADDR_TOTAL_L, st.totalLiters);
  int16_t f; EEPROM.get(ee::ADDR_FAILED_CNT, f); st.failedStartCount = f;
  EEPROM.get(ee::ADDR_PAUSE_MS, st.pauseMs);
  st.savedLiters = st.totalLiters;
}

void saveIfNeeded(unsigned long now) {
  static unsigned long last = 0;
  if (st.totalLiters - st.savedLiters >= 50 || now - last > 600000) {
    EEPROM.put(ee::ADDR_TOTAL_L, st.totalLiters);
    st.savedLiters = st.totalLiters;
    last = now;
  }
}

// =====================================================
// SETUP И ОСНОВНОЙ ЦИКЛ
// =====================================================

void setup() {
  Serial.begin(9600);
  Serial.println(F("\n=== СКВАЖИННЫЙ + ДОМАШНИЙ НАСОС v5.3 ==="));
  Serial.println(F("Добавлена защита по сухому ходу для домашнего насоса"));

  pinMode(RELAY_WELL, OUTPUT);
  digitalWrite(RELAY_WELL, HIGH);
  
  pinMode(PIN_RS485_DE_RE, OUTPUT);
  digitalWrite(PIN_RS485_DE_RE, LOW);

  tft.init(240, 320);
  tft.setRotation(2);
  tft.fillScreen(BLACK);

  // Калибровка тока скважинного насоса
  long sum = 0;
  for (int i = 0; i < 600; i++) { sum += analogRead(ACS_PIN); delay(2); }
  currentZeroOffset = sum / 600.0f;
  if (currentZeroOffset < 400 || currentZeroOffset > 600) currentZeroOffset = 512.0f;

  // Инициализация домашнего насоса
  for (int i = 0; i < hs.SMOOTHING; i++) hs.pressureBuffer[i] = 0.0f;
  
  delay(1000);
  initVFD();

  tft.fillScreen(BLACK);
  drawStatic();

  loadState();

  if (st.intention == PumpIntention::UNKNOWN) {
    st.needPump = false;
    st.targetOk = true;
    st.intention = PumpIntention::TARGET_REACHED;
  }

  st.initStart = millis();
  st.mode = State::Mode::INIT;
}

void loop() {
  unsigned long now = millis();

  // Чтение датчиков скважинного насоса
  readAndFilterLevels(now);
  
  // Пересчитываем needPump КАЖДЫЙ ЦИКЛ по текущим уровням
  updateWellPumpNeed();
  
  st.current = readCurrent();
  st.pressureBar = readPressureBar();

  // Чтение датчиков домашнего насоса
  readHouseSensors();

  // Логика скважинного насоса
  if (st.mode == State::Mode::INIT) {
    if (now - st.initStart >= INIT_DELAY_MS) {
      updateWellPumpNeed();
      st.mode = State::Mode::WAIT;
      st.pauseStart = now;
      st.pauseMs = 0;
      Serial.println("WELL: Initialization complete");
    }
    digitalWrite(RELAY_WELL, HIGH);
  } else {
    runMachine(now);
  }

  // Логика домашнего насоса
  runHousePump(now);

  // Защиты обоих насосов
  applyProtections(now);
  applyHouseProtections(now);

  // Обновление дисплея
  if (now - st.lastUpd >= 1000) {
    st.lastUpd = now;
    updateDisplay(now);
  }

  // Мигание при авариях
  handleFlashing(now);
  
  // Сохранение данных
  saveIfNeeded(now);
}

#endif  // NASOS_V1_INO_GUARD
