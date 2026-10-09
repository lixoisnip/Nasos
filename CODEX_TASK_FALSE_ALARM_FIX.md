# ЗАДАНИЕ ДЛЯ CODEX: фикс ложных аварий по току домашнего насоса + доставка в lixoisnip/Nasos

**Приоритет:** высокий. **Ожидаемое время:** < 1 часа.
**Автономность:** все шаги выполнять самостоятельно; вопросы задавать только если заблокированы правами доступа (в этом случае — см. §7).
**Фикс уже разработан и проверен** (сборки обеих прошивок + симуляция цепи измерения). Задача — воспроизвести его в репозитории и доставить как ветку + PR, ничего не переизобретая.

---

## 1. Контекст

Проект: https://github.com/lixoisnip/Nasos — насосная станция:
- **ESP32** (`esp32_controller.ino`) — главный контроллер: логика насосов и защит, web UI, RS-485/Modbus с VFD, мастер команд/телеметрии Nano;
- **Arduino Nano** (`v1.ino`) — I/O-исполнитель: АЦП (токи, давления, уровни), реле, телеметрия на ESP32 по UART каждые 100 мс.

После PR #92 (коммит `4979dc0`, ветка `acs712-voltage-divider-replacement-aa5a5`) делитель 0–10 В
тока домашнего насоса заменён на **ACS712-30A** (пин A0 Nano). С этого момента домашний насос
ловит **ложные аварии**:

- лог: `Дом: запуск насоса, буст 50 Гц на 3 с` → сразу `Дом: авария — аварийная перегрузка по току` → `FAULT`;
- UI в полном останове: `Ток: 0.2 А`, `Авария: Да`, `Последняя причина остановки: FAULT`.

### Root cause (подробно — в приложении E / файле FALSE_ALARM_ANALYSIS.md)

1. **Nano:** «RMS» тока дома считается окном **12 отсчётов АЦП ≈ 1.25 мс = 1/16 периода 50 Гц**
   (`v1.ino`, `HOUSE_CURRENT_AVG_SAMPLES = 12`, `readAuxAnalog()`). Это не RMS, а мгновенное
   значение: показание = **0.16…1.41 × истинного RMS** в зависимости от фазы окна.
   Старый делитель 0–10 В давал гладкий DC — дефекта не было.
2. **Алиасинг:** телеметрия 100 мс = ровно 5 периодов при 50.0 Гц → во время буста (50 Гц, 3 с)
   фаза окна одинакова в каждом пакете → систематическое завышение ~1.4× весь буст.
3. **ESP32:** фильтр защиты EMA `PROTECTION_FILTER_ALPHA = 0.45` пропускает одиночный выброс
   почти без ослабления, а **аварийная ветка ≥1.5 А трипает мгновенно, без выдержки и без
   маскировки старта** (`ignoreStartCurrent` покрывает только перегрузку 1.3 А/5 с и сухой ход,
   причём 2500 мс из 3000 мс буста). Пусковой ток буста × 1.41 → ложный FAULT.
4. **0.2 А в останове** — шумовой пол ACS712-30A (~10 мВ RMS ≈ 0.15–0.2 А) через несглаженное
   «RMS» и кламп 0.08 А; калибровка нуля только при буте, дрейф не компенсируется.

---

## 2. Целевой результат (Definition of Done)

1. В `lixoisnip/Nasos` создана ветка **`fix/house-current-false-alarms`** от `origin/V22`.
2. В ветке **ровно один коммит** с изменениями строго по **Приложению A**:
   `v1.ino`, `esp32_controller.ino`, новый файл `FALSE_ALARM_ANALYSIS.md`.
3. Обе прошивки компилируются без ошибок (§4.4).
4. Скрипт проверки (**Приложение D**) выводит `ИТОГ: PASS`.
5. Ветка запушена в origin, открыт **PR в базу `V22`** с заголовком и телом по **Приложению C**.
6. Предоставлен отчёт по §8.

**Если нет прав на push/PR:** выполнить пп. 1–4, сохранить патч и ветку локально, в отчёте явно
указать, что п. 5 не выполнен, и приложить инструкции/патч. Не создавать форки и сторонние
репозитории без крайней необходимости.

---

## 3. Запреты и границы

- **Не трогать:** `Osnova.ino`, `data/index.html` (web UI), `diagnostics/`, `i2c_link_config.h`,
  логику скважинного насоса, структуры `NanoTelemetryPayload`/`NanoCommandPayload` и версию
  протокола, ключи и диапазоны настроек (Preferences/Web), логику автоперезапусков, PID, RS-485/VFD.
- **Не добавлять** новые зависимости и библиотеки.
- **Не переписывать историю**, не делать force-push, не пушить ни в какие ветки кроме создаваемой.
- **Не менять константы**, кроме перечисленных в §5.
- Стиль правок — как в Приложении A (комментарии на русском); не переформатировать соседние строки.
- Отступления от Приложения A допустимы только если без них код не компилируется; каждое — в отчёт.

---

## 4. Порядок работы

### 4.1 Подготовка репозитория
```bash
git clone https://github.com/lixoisnip/Nasos.git && cd Nasos
git fetch origin
git checkout -b fix/house-current-false-alarms origin/V22   # база: V22 HEAD (bef853f или новее)
```

### 4.2 Нанесение изменений
**Способ A (основной):** сохранить Приложение A в файл `fix.patch` и применить:
```bash
git am fix.patch
```
**Способ B (если патч не ложится):** воспроизвести изменения вручную строго по Приложению A
(diff дан против `bef853f`; номера строк могут сдвинуться — искать по контексту hunks).

Контроль после нанесения:
```bash
git diff origin/V22 --stat
```
Ожидание: ровно 3 файла — `FALSE_ALARM_ANALYSIS.md` (new), `esp32_controller.ino`, `v1.ino`.

### 4.3 Сверка содержания
```bash
git diff origin/V22 -- v1.ino esp32_controller.ino
```
Должен совпадать с hunks Приложения A (допускаются только сдвigi номеров строк). Свериться также
с таблицей §5.

### 4.4 Сборка (обязательно обе прошивки)
```bash
# arduino-cli, если отсутствует:
arduino-cli config init --overwrite
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install arduino:avr esp32:esp32
arduino-cli lib install ArduinoJson "Adafruit GFX Library" "Adafruit ST7735 and ST7789 Library"

# скетчи: имя папки = имя .ino
mkdir -p build/v1 build/esp32_controller
cp v1.ino build/v1/ && cp esp32_controller.ino build/esp32_controller/
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 build/v1
arduino-cli compile --fqbn esp32:esp32:esp32 build/esp32_controller
```
Ожидание: успешная компиляция обоих; ориентир: Nano ~10.4 КБ flash / ~0.5 КБ RAM,
ESP32 ~1.07 МБ flash. Предупреждения допустимы, ошибки — нет.

### 4.5 Функциональная проверка (симуляция цепи измерения)
Сохранить Приложение D как `verify_simulation.py`, выполнить:
```bash
python3 verify_simulation.py
```
Ожидание: последняя строка `ИТОГ: PASS`. Смысл сценариев:
1) штатный буст 2.0 А/50 Гц 3 с → 0.9 А/38 Гц, реальной перегрузки НЕТ: старая схема трипает
   ложно (0.1 с), новая — не трипает;
2) реальная перегрузка 3.0 А после буста: новая схема трипает (~4.3 с = маскировка старта 4 с + выдержка 250 мс);
3) КЗ 6 А во время буста: новая схема трипает быстро (~0.4 с, hard-short bypass).

### 4.6 Коммит
Способ A: коммит уже создан `git am`. Способ B: создать один коммит с сообщением из Приложения B:
```bash
git add v1.ino esp32_controller.ino FALSE_ALARM_ANALYSIS.md
git commit -F commit_message.txt
```
Проверить: `git rev-list --count origin/V22..HEAD` == 1.

### 4.7 Push и PR
```bash
git push -u origin fix/house-current-false-alarms
# тело PR из Приложения C сохранить в pr_body.md, затем:
gh pr create --base V22 --head fix/house-current-false-alarms \
  --title "Fix false house overcurrent alarms after ACS712-30A migration" \
  --body-file pr_body.md
```
Если `gh` нет — создать PR через GitHub API или Web UI. Если нет токена вовсе — см. §2 (остановиться,
приложить инструкции).

### 4.8 Пост-проверка
- PR открыт, база `V22`, в diff PR ровно 3 файла из Приложения A;
- CI (если есть) зелёный; при красном — см. §7.

---

## 5. Спецификация изменений (сверочная таблица)

| Файл | Место | Было | Стало |
|---|---|---|---|
| v1.ino | константы | `HOUSE_CURRENT_AVG_SAMPLES = 12` | `HOUSE_CURRENT_WINDOW_US = 40000UL`; `HOUSE_CURRENT_EMA_ALPHA = 0.30f`; `HOUSE_ZERO_RECALIB_REST_MS = 5000UL`; `HOUSE_ZERO_ADAPT_ALPHA = 0.02f`; константы HouseMode: WAIT_WATER=0, READY=2, STOPPED=5, FAULT=7 |
| v1.ino | `readAuxAnalog()` | цикл 12 отсчётов, возврат оконного RMS | окно **40 мс** по `micros()` (~380 отсчётов = 2 периода 50 Гц); автокалибровка нуля при остановленном доме ≥5 с (α=0.02, по среднему окна); EMA α=0.30 между окнами; сигнатура `(unsigned long now)`; `feedWatchdog()` в цикле |
| v1.ino | `readInputs()` | `readAuxAnalog()` | `readAuxAnalog(now)` |
| esp32_controller.ino | `houseCtrl` | `START_CURRENT_IGNORE_MS = 2500UL` | `4000UL`; добавлены `EMERGENCY_CONFIRM_MS = 250UL`, `EMERGENCY_START_HARD_LIMIT_MULT = 2.0f` |
| esp32_controller.ino | `houseCurrentSense` | `NEAR_ZERO_CLAMP_A = 0.08f`; `PROTECTION_FILTER_ALPHA = 0.45f` | `0.25f`; `0.20f` |
| esp32_controller.ino | `struct Telemetry` | — | + `float houseCurrentRawHist[2]`, + `float houseCurrentRawLast` |
| esp32_controller.ino | `struct Controller` (`st`) | — | + `unsigned long houseEmergencyStart` |
| esp32_controller.ino | `decodeTelemetryPayload()` | raw → сразу EMA защиты и дисплея | **медиана по 3 пакетам** → EMA защиты и дисплея; `houseCurrentRawLast` = raw без фильтра |
| esp32_controller.ino | `runProtections()`, аварийная ветка дома | мгновенный trip при `tm.houseCurrent >= emergencyCurrent`, без маскировки старта | trip при `(!ignoreStartCurrent || hardShort) && tm.houseCurrent >= emergencyCurrent` **и** выдержке `EMERGENCY_CONFIRM_MS`; `hardShort` = `tm.houseCurrentRawLast >= 2 × emergencyCurrent` трипает мгновенно даже на старте; сброс `houseEmergencyStart` при токе ниже порога |
| esp32_controller.ino | все 5 точек сброса `st.houseOverloadStart` (runHouseAutoRestart, trip перегрузки, trip сухого хода, else-ветвь перегрузки, else-ветвь `if (st.vfdRun)`) | — | + `st.houseEmergencyStart = 0;` |

Поведенческие требования:
- формат телеметрии НЕ меняется: `analogAuxRaw` = RMS в отсчётах АЦП (целое), конвертация ESP32 `convertNanoAnalogToCurrent()` без изменений;
- аварийная ветка не должна срабатывать от одиночных выбросов длительностью < 250 мс и в первые 4 с после старта (кроме hard short);
- защиты «перегрузка 1.3 А / 5 с» и «сухой ход» работают как раньше (их ветки не менять, кроме добавления сброса `houseEmergencyStart`).

---

## 6. Критерии приёмки (чек-лист в отчёт)

- [ ] ветка `fix/house-current-false-alarms` создана от `origin/V22`;
- [ ] ровно 1 коммит поверх базы;
- [ ] diff с базой = Приложение A (3 файла, без посторонних правок);
- [ ] сборка `arduino:avr:nano:cpu=atmega328`: OK;
- [ ] сборка `esp32:esp32:esp32`: OK;
- [ ] `verify_simulation.py`: `ИТОГ: PASS`;
- [ ] ветка запушена;
- [ ] PR открыт в `V22`, заголовок/тело по Приложению C;
- [ ] запретов §3 не нарушено.

---

## 7. Обработка отказов

| Ситуация | Действие |
|---|---|
| `git am` не применяет патч | Способ B (ручное воспроизведение по Приложению A) |
| Ошибки компиляции после способа B | Искать свою опечатку при воспроизведении; специфику §5 и Приложение A не менять |
| Нет прав на push/PR | Выполнить §4.1–4.5, в отчёте приложить патч (`git format-patch -1`) и инструкции по пушу/PR |
| CI красный на PR | Разобраться; правки только в рамках Приложения A; отразить в отчёте |
| Необъяснимое расхождение симуляции с ожиданием | Не менять пороги «под результат»; сообщить в отчёте с выводами симуляции |

---

## 8. Формат отчёта

1. Ссылка на PR (или причина отсутствия + приложенный патч/инструкции);
2. SHA коммита;
3. Последние 3 строка вывода каждой сборки;
4. Полный вывод `verify_simulation.py`;
5. Заполненный чек-лист §6;
6. Список отступлений от Приложения A (по умолчанию — пуст).

---

## Приложения

- **A** — полный diff фикса (`fix.patch`): 3 файла, применять как есть.
- **B** — сообщение коммита.
- **C** — заголовок и тело PR.
- **D** — `verify_simulation.py` (скрипт проверки).
- **E** — `FALSE_ALARM_ANALYSIS.md` (разбор root cause с номерами строк; добавляется в репозиторий этим же коммитом).

---

## Приложение A — полный diff фикса (файл fix.patch)

```diff
From 76cb6b61b8e7fa7b7cd5a0336a4f94c5e39fdd0b Mon Sep 17 00:00:00 2001
From: Qwen <qwen@local>
Date: Thu, 8 Oct 2026 11:07:51 +0000
Subject: [PATCH] Fix false house overcurrent alarms after ACS712-30A migration

- v1.ino: house current RMS over 40 ms time window (2 periods of 50 Hz)
  instead of 12-sample (~1.25 ms) window that returned instantaneous value
  (0.16..1.41 x true RMS); add inter-window EMA and zero-offset
  recalibration while house pump is stopped
- esp32: mask emergency overcurrent branch during start (4 s, was unmasked),
  add 250 ms confirmation (instant trip kept only for >=2x hard short),
  median-of-3 packets before protection EMA, alpha 0.45 -> 0.20,
  near-zero clamp 0.08 -> 0.25 A
- verified: both sketches compile (avr:nano, esp32:esp32); simulation shows
  old scheme trips at 0.1 s on normal 2 A start boost, new scheme does not,
  while real 3 A overload and 6 A short still trip
---
 FALSE_ALARM_ANALYSIS.md | 109 ++++++++++++++++++++++++++++++++++++++++
 esp32_controller.ino    |  60 +++++++++++++++++-----
 v1.ino                  |  60 +++++++++++++++++++---
 3 files changed, 208 insertions(+), 21 deletions(-)
 create mode 100644 FALSE_ALARM_ANALYSIS.md

diff --git a/FALSE_ALARM_ANALYSIS.md b/FALSE_ALARM_ANALYSIS.md
new file mode 100644
index 0000000..ece6656
--- /dev/null
+++ b/FALSE_ALARM_ANALYSIS.md
@@ -0,0 +1,109 @@
+# Причина ложных аварий «Дом: авария — аварийная перегрузка по току»
+
+Ветка: `acs712-voltage-divider-replacement-aa5a5` (коммит `4979dc0` — замена делителя 0–10 В
+на ACS712-30A на пине A0 Nano).
+
+## Коротко
+
+Авария ложная, потому что «RMS» тока дома измеряется окном **12 отсчётов АЦП ≈ 1.25 мс**
+(`v1.ino:89, 204–212`) — это **1/16 периода 50 Гц**. Такое «RMS» на самом деле является
+мгновенным значением тока и в зависимости от фазы даёт **от 0.16 до 1.41 × Iист**.
+Пик-выбросы попадают в телеметрию (100 мс), фильтр защиты на ESP32 (EMA α=0.45,
+`esp32_controller.ino:1315`) пропускает одиночный выброс почти без ослабления, а ветка
+**EMERGENCY (≥1.5 А) срабатывает мгновенно, без задержки и без маскировки старта**
+(`esp32_controller.ino:1857`), в то время как буст-старт длится 3 с на 50 Гц — именно там
+ток наибольший (пусковой). Итог: каждый неудачный старт → «аварийная перегрузка по току».
+
+## Цепочка ошибки (с номерами строк)
+
+1. **Nano, `v1.ino:89`** — `HOUSE_CURRENT_AVG_SAMPLES = 12`.
+   `analogRead` на AVR Nano ≈ 104 мкс (prescaler 128, 13 тактов) → окно ≈ **1.25 мс = 22.5°**
+   периода 50 Гц. Среднее sin² по окну = ½ + sin(W)·cos(2φ+W)/(2W), |sinW|/(2W) = 0.487 →
+   показание = √2·I·√(среднее sin²) ∈ **[0.16·I … 1.41·I]** в зависимости от фазы φ.
+   Для сравнения: скважина меряется 120 отсчётами (`v1.ino:88`), а старый делитель 0–10 В
+   давал вообще гладкий DC-сигнал — проблемы не существовало.
+2. **Nano, `v1.ino:449–455`** — телеметрия каждые 100 мс несёт одно такое «оконное» значение
+   без дополнительного сглаживания.
+3. **Алиасинг/фазовая синхронизация**: 100 мс = ровно 5 периодов при 50.0 Гц. Во время
+   буста (`houseCtrl::BOOST_FREQ = 50.0`) фаза окна **одинакова в каждом пакете** → если фаза
+   попала на горб синусоиды, все пакеты в течение всего буста систематически завышены в ~1.4
+   раза (нет усреднения по пакетам).
+4. **ESP32, `esp32_controller.ino:1315`** — `PROTECTION_FILTER_ALPHA = 0.45`: один пакет с
+   выбросом S сдвигает защитное значение на 45 % к S. При prev ≈ 1.0 А для пробоя порога
+   1.5 А достаточно одного пакета с S ≥ (1.5 − 0.55·1.0)/0.45 ≈ **2.04 А** — пусковой ток
+   буста, умноженный на 1.41, легко это даёт.
+5. **ESP32, `esp32_controller.ino:1855–1864`** — ключевая логическая ошибка:
+   `ignoreStartCurrent` (первые 2500 мс после старта) применяется к перегрузке 1.3 А/5 с
+   (стр. 1866) и сухому ходу (стр. 1899), но **НЕ к аварийной ветке ≥1.5 А (стр. 1857)**,
+   у которой к тому же нет ни выдержки времени, ни подтверждения N пакетами.
+   Дополнительно: 2500 мс маскировки < 3000 мс буста (`houseCtrl:295–297`) — последние
+   0.5 с буста не маскированы даже для перегрузки.
+6. Результат в логах (скриншот): `запуск насоса, буст 50 Гц на 3 с` → сразу
+   `авария — аварийная перегрузка по току` → `FAULT`, `Ток: 0.2 А` в останове.
+
+## Сопутствующие симптомы того же дефекта
+
+- **Ток 0.2 А в полном останове** (на скриншоте): шумовой пол ACS712-30A
+  (~155 мкВ/√Гц · √4.8 кГц ≈ 10 мВ RMS ≈ 0.15–0.2 А) попадает в несглаженное «RMS».
+  Клампы `<0.1 А` (Nano) и `NEAR_ZERO_CLAMP 0.08 А` (ESP32) его не срезают.
+  Калибровка нуля выполняется только при старте (`v1.ino:548–556`) — дрейф не компенсируется.
+- **Дёрганое показание тока на UI** во время работы (фаза окна плавает при f ≠ 50 Гц:
+  при 38 Гц сдвиг фазы 288°/пакет → каждый 5-й пакет близок к пику).
+- В `Osnova.ino:343–357` тот же класс ошибки: окно 120 отсчётов = 12.5 мс = 0.625 периода
+  (±9 % на 50 Гц, ±17 % на 28 Гц) и мгновенный аварийный_trip без сглаживания
+  (у скважины хотя бы есть EMA в `readCurrent()`).
+
+## Что чинить
+
+1. **Nano (`v1.ino`)**: окно RMS по времени, а не по 12 отсчётам — ≥ 2 периодов
+   (например, набор отсчётов в течение 60 мс ≈ 300–600 шт., или детект нуля и целое число
+   периодов) + EMA на самой Nano (α ≈ 0.3). Это убирает разброс 0.16…1.41 и фазовую
+   синхронизацию с телеметрий.
+2. **ESP32 (`esp32_controller.ino:1857`)**: распространить `ignoreStartCurrent` на аварийную
+   ветку И увеличить окно до ≥ буста + запас (3500–4000 мс); аварийной ветке добавить
+   подтверждение (2–3 пакета подряд или выдержку 150–250 мс).
+3. **ESP32 (`esp32_controller.ino:1315`)**: уменьшить `PROTECTION_FILTER_ALPHA` до 0.15–0.25
+   или вставить медианный фильтр по 3 пакетам перед EMA — одиночный выброс перестанет
+   пробивать порог.
+4. **Ноль/шум**: перекалибровка нуля в останове (при `vfdRun == false`, скользящим средним),
+   поднять `NEAR_ZERO_CLAMP_A` до ~0.25 А, чтобы шумовой пол 0.15–0.2 А не светился как ток.
+5. Долгосрочно: ACS712-30A (66 мВ/А) для рабочего тока 0.6–1.0 А — плохое отношение
+   сигнал/шум; рассмотреть ACS711/трансформатор тока или аппаратный ФНЧ ~100–500 Гц
+   перед АЦП.
+
+Пункты 1–3 устраняют причину ложных срабатываний; пункт 4 убирает «0.2 А в останове».
+
+---
+
+# Применённые правки (ветка `fix/house-current-false-alarms`)
+
+## `v1.ino` (Nano)
+- `HOUSE_CURRENT_AVG_SAMPLES = 12` заменено на **временное окно 40 мс**
+  (`HOUSE_CURRENT_WINDOW_US = 40000`, ~380 отсчётов = 2 периода 50 Гц, >1 периода 28 Гц)
+  + EMA α=0.30 между окнами (`HOUSE_CURRENT_EMA_ALPHA`). Разброс 0.16…1.41 устранён.
+- Добавлена **автокалибровка нуля в останове**: при `houseMode` ∈ {WAIT_WATER, READY,
+  STOPPED, FAULT} дольше 5 с смещение нуля медленно подстраивается
+  (`HOUSE_ZERO_ADAPT_ALPHA = 0.02`) — компенсирует дрейф, убирет «0.2 А в останове».
+- Формат телеметрии (`analogAuxRaw` = RMS в отсчётах АЦП) и конвертация на ESP32 НЕ менялись.
+
+## `esp32_controller.ino`
+- `houseCtrl::START_CURRENT_IGNORE_MS`: 2500 → **4000 мс** (покрывает буст 3000 мс + запас).
+- Аварийная ветка ≥1.5 А (`runProtections`): теперь **маскируется на старте** и требует
+  **выдержки `EMERGENCY_CONFIRM_MS = 250 мс`**; мгновенный trip сохранён только для
+  жёсткого КЗ (`EMERGENCY_START_HARD_LIMIT_MULT = 2.0`, по необработанному отсчёту
+  `tm.houseCurrentRawLast`).
+- `houseCurrentSense::PROTECTION_FILTER_ALPHA`: 0.45 → **0.20**; перед фильтром добавлен
+  **медианный фильтр по 3 пакетам** (`tm.houseCurrentRawHist`) — одиночный выброс больше
+  не пробивает порог.
+- `houseCurrentSense::NEAR_ZERO_CLAMP_A`: 0.08 → **0.25** (шумовой пол ACS712-30A
+  ~0.15–0.2 А не отображается как ток).
+- Добавлены поля `tm.houseCurrentRawHist[2]`, `tm.houseCurrentRawLast`,
+  `st.houseEmergencyStart` (сбрасывается во всех точках сброса `houseOverloadStart`).
+
+## Проверка
+- `arduino-cli compile --fqbn arduino:avr:nano` (v1.ino): OK, 10444 B flash / 491 B RAM.
+- `arduino-cli compile --fqbn esp32:esp32:esp32` (esp32_controller.ino): OK, 1068144 B flash.
+- Симуляция цепи измерения (окно АЦП 104 мкс, телеметрия 100 мс, буст 2.0 А/50 Гц 3 с,
+  далее 0.9 А/38 Гц): **старая схема — trip на 0.1 с при большинстве фаз окна (ложно);
+  новая схема — trip нет**. Контроль защит: реальная перегрузка 3.0 А после буста —
+  new trip 4.3 с (маскировка старта + выдержка); КЗ 6 А на старте — new trip 0.4 с.
diff --git a/esp32_controller.ino b/esp32_controller.ino
index 6d41a21..4910d9c 100644
--- a/esp32_controller.ino
+++ b/esp32_controller.ino
@@ -292,9 +292,11 @@ constexpr float PRESSURE_RISE_THRESHOLD = 0.10f;
 constexpr unsigned long DRY_PRESSURE_START_TIMEOUT = 10000UL;
 constexpr unsigned long DRY_PRESSURE_WORK_TIMEOUT = 15000UL;
 
-constexpr unsigned long START_CURRENT_IGNORE_MS = 2500UL;
+constexpr unsigned long START_CURRENT_IGNORE_MS = 4000UL; // было 2500: буст длится 3000 мс, маскировка должна покрывать его целиком + запас
 constexpr float BOOST_FREQ = 50.0f;
 constexpr unsigned long BOOST_DURATION_MS = 3000UL;
+constexpr unsigned long EMERGENCY_CONFIRM_MS = 250UL;     // выдержка аварийной ветки: защита от одиночных выбросов
+constexpr float EMERGENCY_START_HARD_LIMIT_MULT = 2.0f;   // во время старта мгновенный trip только при >= 2x emergency (короткое замыкание)
 constexpr uint8_t AUTO_RESTART_MAX = 3;               // Intentional deviation: Osnova had no dedicated auto-restart counter for house faults; capped retries prevent endless cycling. Risk: repeated attempts can still stress motor during persistent fault.
 constexpr unsigned long AUTO_RESTART_DELAY_MS = 2000UL; // Intentional deviation: short cooldown to restore household pressure quickly after transient trips. Risk: if source fault persists, retries happen sooner.
 constexpr unsigned long RESTART_RESET_OK_MS = 10UL * 60UL * 1000UL;
@@ -320,8 +322,8 @@ constexpr float ACS712_30A_SENSITIVITY_V_PER_A = 0.066f;  // Чувствите
 constexpr float CURRENT_PER_ADC_VOLT = 1.0f / ACS712_30A_SENSITIVITY_V_PER_A;  // current(A) = adcVoltage(V) / 0.066
 constexpr float CURRENT_CALIBRATION_GAIN = 1.0f;
 constexpr float CURRENT_CALIBRATION_OFFSET = 0.0f;
-constexpr float NEAR_ZERO_CLAMP_A = 0.08f;
-constexpr float PROTECTION_FILTER_ALPHA = 0.45f;
+constexpr float NEAR_ZERO_CLAMP_A = 0.25f;      // было 0.08: шумовой пол ACS712-30A ~0.15-0.2 А не должен светиться как ток
+constexpr float PROTECTION_FILTER_ALPHA = 0.20f; // было 0.45: одиночный выброс не должен пробивать порог аварии
 constexpr float DISPLAY_FILTER_ALPHA = 0.20f;
 constexpr float MAX_SANE_CURRENT_A = 30.0f;  // ACS712-30A: диапазон до 30 А
 }
@@ -419,6 +421,8 @@ struct Telemetry {
   float houseCurrent = 0;
   float houseCurrentDisplay = 0;
   float houseCurrentProtection = 0;
+  float houseCurrentRawHist[2] = {0, 0};  // история для медианного фильтра 3 пакетов
+  float houseCurrentRawLast = 0;          // последний необработанный отсчёт (для мгновенного trip по КЗ)
   float housePressure = 0;
   bool levels[4] = {false, false, false, false};
   bool vfdRunFeedback = false;
@@ -501,6 +505,7 @@ struct Controller {
   unsigned long wellOverloadStart = 0;
   unsigned long houseDryStart = 0;
   unsigned long houseOverloadStart = 0;
+  unsigned long houseEmergencyStart = 0;
   unsigned long houseStartAt = 0;
   unsigned long houseLastStopAt = 0;
   unsigned long houseBoostStartAt = 0;
@@ -1312,8 +1317,22 @@ bool decodeTelemetryPayload(const NanoTelemetryPayload& payload, unsigned long n
   tm.wellCurrent = normalizeTelemetryCurrent(payload.wellCurrentCentiA / 100.0f, telemetryCurrent::WELL_GAIN);
   tm.wellPressure = payload.wellPressureCentiBar / 100.0f;
   const float houseCurrentRaw = convertNanoAnalogToCurrent((uint16_t)max(0, (int)payload.analogAuxRaw));
-  tm.houseCurrentProtection += houseCurrentSense::PROTECTION_FILTER_ALPHA * (houseCurrentRaw - tm.houseCurrentProtection);
-  tm.houseCurrentDisplay += houseCurrentSense::DISPLAY_FILTER_ALPHA * (houseCurrentRaw - tm.houseCurrentDisplay);
+  // Медиана по 3 последним пакетам: гасит одиночные выбросы «оконного» RMS Nano
+  // до того, как значение попадёт в фильтр защиты (раньше один выброс почти
+  // полностью пробивал порог 1.5 А через EMA alpha=0.45).
+  float a = tm.houseCurrentRawHist[0];
+  float b = tm.houseCurrentRawHist[1];
+  float c = houseCurrentRaw;
+  tm.houseCurrentRawHist[0] = b;
+  tm.houseCurrentRawHist[1] = c;
+  tm.houseCurrentRawLast = houseCurrentRaw;
+  float t;
+  if (a > b) { t = a; a = b; b = t; }
+  if (b > c) { t = b; b = c; c = t; }
+  if (a > b) { t = a; a = b; b = t; }
+  const float houseCurrentFilt = b;
+  tm.houseCurrentProtection += houseCurrentSense::PROTECTION_FILTER_ALPHA * (houseCurrentFilt - tm.houseCurrentProtection);
+  tm.houseCurrentDisplay += houseCurrentSense::DISPLAY_FILTER_ALPHA * (houseCurrentFilt - tm.houseCurrentDisplay);
   if (tm.houseCurrentProtection < houseCurrentSense::NEAR_ZERO_CLAMP_A) tm.houseCurrentProtection = 0.0f;
   if (tm.houseCurrentDisplay < houseCurrentSense::NEAR_ZERO_CLAMP_A) tm.houseCurrentDisplay = 0.0f;
   tm.houseCurrent = tm.houseCurrentProtection;
@@ -1807,6 +1826,7 @@ void runHouseAutoRestart(unsigned long now) {
   st.houseTargetFreq = constrain(houseCtrl::BOOST_FREQ, cfg.house.minFreq, cfg.house.maxFreq);
   st.vfdFreq = st.houseTargetFreq;
   st.houseOverloadStart = 0;
+  st.houseEmergencyStart = 0;
   st.houseDryStart = 0;
 
   appendLog(st.logsHouse, "Дом: автоперезапуск " + String(st.houseAutoRestartAttempts) + "/" + String(houseCtrl::AUTO_RESTART_MAX));
@@ -1853,14 +1873,25 @@ void runProtections(unsigned long now) {
 
   if (st.vfdRun) {
     bool ignoreStartCurrent = st.houseStartAt && (now - st.houseStartAt < houseCtrl::START_CURRENT_IGNORE_MS);
-
-    if (tm.houseCurrent >= cfg.house.emergencyCurrent) {
-      st.houseBlocked = st.houseAlarm = true;
-      st.vfdRun = false;
-      st.houseManualMode = ManualMode::AUTO;
-      st.houseMode = HouseMode::FAULT;
-      st.houseLastStopReason = HouseStopReason::FAULT;
-      appendLog(st.logsHouse, "Дом: авария — аварийная перегрузка по току");
+    const bool hardShort = tm.houseCurrentRawLast >= cfg.house.emergencyCurrent * houseCtrl::EMERGENCY_START_HARD_LIMIT_MULT;
+
+    // Аварийная перегрузка: раньше trip был мгновенным и НЕ маскировался на старте,
+    // поэтому пусковой ток буста (плюс выбросы оконного RMS) ронял насос в FAULT.
+    // Теперь: во время старта маскируется (кроме жёсткого КЗ >= 2x порога),
+    // и в любом случае требует выдержки EMERGENCY_CONFIRM_MS.
+    if ((!ignoreStartCurrent || hardShort) && tm.houseCurrent >= cfg.house.emergencyCurrent) {
+      if (st.houseEmergencyStart == 0) st.houseEmergencyStart = now;
+      if (hardShort || (now - st.houseEmergencyStart >= houseCtrl::EMERGENCY_CONFIRM_MS)) {
+        st.houseBlocked = st.houseAlarm = true;
+        st.vfdRun = false;
+        st.houseManualMode = ManualMode::AUTO;
+        st.houseMode = HouseMode::FAULT;
+        st.houseLastStopReason = HouseStopReason::FAULT;
+        st.houseEmergencyStart = 0;
+        appendLog(st.logsHouse, "Дом: авария — аварийная перегрузка по току");
+      }
+    } else if (tm.houseCurrent < cfg.house.emergencyCurrent) {
+      st.houseEmergencyStart = 0;
     }
 
     if (!ignoreStartCurrent && tm.houseCurrent >= cfg.house.overloadCurrent) {
@@ -1873,6 +1904,7 @@ void runProtections(unsigned long now) {
         st.houseLastStopReason = HouseStopReason::FAULT;
         st.houseLastStopReason = HouseStopReason::FAULT;
         st.houseOverloadStart = 0;
+        st.houseEmergencyStart = 0;
         st.houseDryStart = 0;
         st.housePressureDryStartAt = 0;
         st.houseStartAt = 0;
@@ -1904,6 +1936,7 @@ void runProtections(unsigned long now) {
         st.houseMode = HouseMode::STOPPED;
         st.houseAlarm = true;
         st.houseOverloadStart = 0;
+        st.houseEmergencyStart = 0;
         st.houseDryStart = 0;
         st.housePressureDryStartAt = 0;
         st.houseStartAt = 0;
@@ -1942,6 +1975,7 @@ void runProtections(unsigned long now) {
     }
   } else {
     st.houseOverloadStart = 0;
+    st.houseEmergencyStart = 0;
     st.houseDryStart = 0;
     st.housePressureDryStartAt = 0;
     st.houseStartAt = 0;
diff --git a/v1.ino b/v1.ino
index 70bfce8..068a4a6 100644
--- a/v1.ino
+++ b/v1.ino
@@ -86,7 +86,18 @@ struct NanoTelemetryPayload {
 } __attribute__((packed));
 
 const uint8_t WELL_CURRENT_SAMPLES = 120;
-const uint8_t HOUSE_CURRENT_AVG_SAMPLES = 12;
+// Окно RMS тока дома задаётся ВРЕМЕНЕМ, а не числом отсчётов: 40 мс = 2 периода 50 Гц
+// и > 1 периода 28 Гц. Прежние 12 отсчётов (~1.25 мс = 1/16 периода) возвращали
+// мгновенное значение (0.16..1.41 от истинного RMS) -> ложные аварии.
+const unsigned long HOUSE_CURRENT_WINDOW_US = 40000UL;
+const float HOUSE_CURRENT_EMA_ALPHA = 0.30f;         // сглаживание между окнами
+const unsigned long HOUSE_ZERO_RECALIB_REST_MS = 5000UL;  // перебазировка нуля после >=5 с покоя
+const float HOUSE_ZERO_ADAPT_ALPHA = 0.02f;          // скорость подстройки нуля
+// Значения HouseMode с ESP32 (enum class HouseMode, esp32_controller.ino)
+const int HOUSE_MODE_WAIT_WATER = 0;
+const int HOUSE_MODE_READY = 2;
+const int HOUSE_MODE_STOPPED = 5;
+const int HOUSE_MODE_FAULT = 7;
 const uint8_t HOUSE_PRESSURE_AVG_SAMPLES = 6;
 const unsigned long LEVEL_FILTER_MS_DEFAULT = 2000;
 const int LEVEL_THRESH_DEFAULT = 700;
@@ -199,16 +210,49 @@ float readWellCurrent() {
 
 // Домашний насос: ACS712-30A на PIN_CURRENT (A0) — RMS тока вокруг нуля датчика.
 // Формула подобрана так, чтобы значение, передаваемое в поле analogAuxRaw,
-// проходило через НЕИЗМЕННУЮ конвертацию ESP32 (V*2.0) и давало реальные амперы:
+// проходило через НЕИЗМЕННУЮ конвертацию ESP32 (V/0.066) и давало реальные амперы:
 // amps = rmsVoltage / 0.066;  analogAuxRaw = amps * 0.066 * 1023 / 5.0 (= "эквивалентное напряжение" АЦП)
-float readAuxAnalog() {
+// Окно набора — фиксированные 40 мс (>= 2 периодов 50 Гц): окно из 12 отсчётов
+// (~1.25 мс) измеряло мгновенное значение и давало разброс 0.16..1.41 от RMS,
+// что приводило к ложным срабатываниям аварийной защиты по току.
+float readAuxAnalog(unsigned long now) {
+  // Автокалибровка нуля: отслеживаем дрейф смещения только при остановленном
+  // доме (VFD выключен >= HOUSE_ZERO_RECALIB_REST_MS), чтобы не захватить
+  // спадающий ток после останова и не исказить ноль под нагрузкой.
+  static unsigned long houseStoppedSince = 0;
+  const bool houseStopped = (ns.houseMode == HOUSE_MODE_WAIT_WATER ||
+                             ns.houseMode == HOUSE_MODE_READY ||
+                             ns.houseMode == HOUSE_MODE_STOPPED ||
+                             ns.houseMode == HOUSE_MODE_FAULT);
+  if (houseStopped) {
+    if (houseStoppedSince == 0) houseStoppedSince = now;
+  } else {
+    houseStoppedSince = 0;
+  }
+
+  const unsigned long t0 = micros();
   long sumSq = 0;
-  for (uint8_t i = 0; i < HOUSE_CURRENT_AVG_SAMPLES; i++) {
-    float delta = analogRead(PIN_CURRENT) - ns.houseCurrentZeroOffset;
+  long sumRaw = 0;
+  uint16_t n = 0;
+  while (micros() - t0 < HOUSE_CURRENT_WINDOW_US) {
+    const int raw = analogRead(PIN_CURRENT);
+    const float delta = raw - ns.houseCurrentZeroOffset;
     sumSq += (long)(delta * delta);
+    sumRaw += raw;
+    n++;
+    feedWatchdog();
   }
-  float rmsRaw = sqrt((float)sumSq / HOUSE_CURRENT_AVG_SAMPLES);
-  return constrain(rmsRaw, 0.0f, 1023.0f);
+  if (n == 0) return ns.houseCurrent;
+
+  if (houseStoppedSince != 0 && (now - houseStoppedSince) >= HOUSE_ZERO_RECALIB_REST_MS) {
+    const float meanRaw = (float)sumRaw / (float)n;
+    ns.houseCurrentZeroOffset += HOUSE_ZERO_ADAPT_ALPHA * (meanRaw - ns.houseCurrentZeroOffset);
+  }
+
+  const float rmsRaw = sqrt((float)sumSq / (float)n);
+  // EMA между окнами: гасит остаточную нецелократность окна и фазовые биения
+  ns.houseCurrent = HOUSE_CURRENT_EMA_ALPHA * rmsRaw + (1.0f - HOUSE_CURRENT_EMA_ALPHA) * ns.houseCurrent;
+  return constrain(ns.houseCurrent, 0.0f, 1023.0f);
 }
 
 float readWellPressureBar() {
@@ -261,7 +305,7 @@ bool readLevelFiltered(uint8_t idx, uint8_t pin, unsigned long now) {
 
 void readInputs(unsigned long now) {
   ns.wellCurrent = readWellCurrent();
-  ns.houseCurrent = readAuxAnalog();
+  ns.houseCurrent = readAuxAnalog(now);
   ns.wellPressureBar = readWellPressureBar();
   ns.housePressureBar = readHousePressureBar();
 
-- 
2.39.5

```

## Приложение B — сообщение коммита (файл commit_message.txt)

```
Fix false house overcurrent alarms after ACS712-30A migration

- v1.ino: house current RMS over 40 ms time window (2 periods of 50 Hz)
  instead of 12-sample (~1.25 ms) window that returned instantaneous value
  (0.16..1.41 x true RMS); add inter-window EMA and zero-offset
  recalibration while house pump is stopped
- esp32: mask emergency overcurrent branch during start (4 s, was unmasked),
  add 250 ms confirmation (instant trip kept only for >=2x hard short),
  median-of-3 packets before protection EMA, alpha 0.45 -> 0.20,
  near-zero clamp 0.08 -> 0.25 A
- verified: both sketches compile (avr:nano, esp32:esp32); simulation shows
  old scheme trips at 0.1 s on normal 2 A start boost, new scheme does not,
  while real 3 A overload and 6 A short still trip

```

## Приложение C — заголовок и тело PR (файл pr_body.md)

```markdown
# Fix false house overcurrent alarms after ACS712-30A migration

## Summary

After replacing the house-pump 0–10 V current divider with an ACS712-30A (PR #92), the house
pump started tripping `Дом: авария — аварийная перегрузка по току` on normal starts (log:
`запуск насоса, буст 50 Гц на 3 с` → immediate FAULT), and showed ~0.2 A while fully stopped.
This PR fixes the measurement chain and the protection logic; no telemetry protocol changes.

## Root cause

1. **Nano (`v1.ino`)**: house current "RMS" was computed over **12 consecutive ADC samples
   (~1.25 ms = 1/16 of a 50 Hz period)**. Such a window returns a phase-dependent
   quasi-instantaneous value: **0.16…1.41 × true RMS**. The legacy 0–10 V divider supplied a
   smooth DC signal, so the defect appeared only with the ACS712.
2. **Sampling aliasing**: telemetry period is 100 ms = exactly 5 periods at 50.0 Hz, so during
   the 50 Hz start boost every packet hit the *same* sine phase — a persistent ~1.4× overread
   for the whole 3 s boost instead of averaging out.
3. **ESP32 (`esp32_controller.ino`)**: the protection EMA (`PROTECTION_FILTER_ALPHA = 0.45`)
   let a single spike packet move the filtered value 45 % toward the spike, and the
   **emergency branch (≥ 1.5 A) tripped instantly, with no confirmation delay and without the
   `ignoreStartCurrent` masking** (which covered only the 1.3 A/5 s overload and dry-run, and
   only for 2.5 s of the 3 s boost). Start-boost inrush × 1.41 therefore landed in FAULT.
4. The 0.2 A at standstill is the ACS712-30A noise floor (~10 mV RMS ≈ 0.15–0.2 A) passing
   through the unfiltered RMS and the 0.08 A near-zero clamp.

Full analysis with line references: `FALSE_ALARM_ANALYSIS.md` (added by this PR).

## Changes

### `v1.ino` (Nano)
- House current RMS over a **40 ms time window** (~380 samples = 2 periods of 50 Hz,
  > 1 period at 28 Hz) instead of 12 samples; inter-window EMA α = 0.30.
- **Zero-offset recalibration in standstill**: when `houseMode` ∈ {WAIT_WATER, READY, STOPPED,
  FAULT} for ≥ 5 s, the offset slowly adapts (α = 0.02) — tracks drift, removes the 0.2 A rest
  reading.
- Telemetry format unchanged (`analogAuxRaw` = RMS in ADC counts); ESP32 conversion untouched.

### `esp32_controller.ino`
- `houseCtrl::START_CURRENT_IGNORE_MS`: 2500 → **4000 ms** (covers the whole 3 s boost + margin).
- Emergency branch (≥ `emergencyCurrent`): now **masked during start** and requires
  **`EMERGENCY_CONFIRM_MS = 250 ms`** of sustained overcurrent; instant trip kept only for a
  hard short (≥ 2× threshold, evaluated on the raw sample `tm.houseCurrentRawLast`).
- **Median-of-3 packets** before the protection filter; `PROTECTION_FILTER_ALPHA` 0.45 → **0.20**
  — a single outlier packet can no longer punch through the 1.5 A threshold.
- `NEAR_ZERO_CLAMP_A` 0.08 → **0.25** (sensor noise floor no longer displayed as current).
- New state: `tm.houseCurrentRawHist[2]`, `tm.houseCurrentRawLast`, `st.houseEmergencyStart`
  (reset everywhere `houseOverloadStart` is reset).

## Verification

- `arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328` (v1.ino): OK — 10 444 B flash,
  491 B RAM.
- `arduino-cli compile --fqbn esp32:esp32:esp32` (esp32_controller.ino): OK — 1 068 144 B flash.
- Measurement-chain simulation (104 µs/sample ADC, 100 ms telemetry, 2.0 A inrush @ 50 Hz for
  3 s → 0.9 A @ 38 Hz, no real fault): **old scheme trips at 0.1 s for most window phases
  (false alarm); new scheme never trips**. Protection sanity: real 3.0 A overload after boost →
  trip at 4.3 s (start mask + confirmation); 6 A short during start → trip at 0.4 s
  (hard-short bypass works).

## Risk / rollback

Single commit; protections remain fully functional (overload 1.3 A/5 s and dry-run paths are
unchanged except timer resets). Rollback = revert the commit.
```

## Приложение D — verify_simulation.py

```python
#!/usr/bin/env python3
"""Проверка цепи измерения тока домашнего насоса: старая схема vs новая.

Модель: АЦП Nano 104 мкс/отсчёт, телеметрия 100 мс, ACS712-30A 0.066 В/А,
передача RMS в отсчётах АЦП (целое), фильтры ESP32 как в прошивке.

Сценарии:
 1) штатный старт: буст 2.0 А / 50 Гц / 3 c, далее 0.9 А / 38 Гц  -> аварий быть НЕ должно;
 2) реальная перегрузка 3.0 А после буста                          -> авария ОБЯЗАНА;
 3) КЗ 6 А во время буста                                          -> авария ОБЯЗАНА (hard short).

Ожидаемый результат после фикса:
  сценарий 1: old = trip (ложно), new = None
  сценарий 2: new = trip ~4.3 c (маскировка старта 4 c + выдержка 250 мс)
  сценарий 3: new = trip ~0.4 c (мгновенный trip по hard short >= 2x порога)
"""
import math

ADC_US = 104e-6          # длительность одного analogRead на AVR Nano (prescaler 128)
T_TELEM = 0.1            # период телеметрии Nano -> ESP32, с


def counts_from_amps(a):
    return round(a * 0.066 * 1023 / 5.0)


def amps_from_counts(c):
    return c * 5 / 1023 / 0.066


def window_rms(I, f, phi0, n):
    """RMS синусоиды тока по окну из n подряд идущих отсчётов АЦП."""
    T = n * ADC_US
    N = 400
    s = 0.0
    for k in range(N):
        t = (k + 0.5) / N * T
        s += (math.sqrt(2) * I * math.sin(2 * math.pi * f * t + phi0)) ** 2
    return math.sqrt(s / N)


def scenario(scheme, I_inrush=2.0, I_work=0.9, phi0=math.pi / 2, true_overload=False):
    """scheme='old' — прошивка до фикса, 'new' — после фикса. Возвращает время trip или None."""
    ema = 0.0
    nano_ema = 0.0
    hist = [0.0, 0.0]
    em_t0 = None
    nwin = 12 if scheme == "old" else int(0.040 / ADC_US)   # 12 отсчётов vs окно 40 мс
    alpha = 0.45 if scheme == "old" else 0.20              # PROTECTION_FILTER_ALPHA
    clamp = 0.08 if scheme == "old" else 0.25              # NEAR_ZERO_CLAMP_A
    t = 0.0
    while t <= 8.0:
        f = 50.0 if t < 3.0 else 38.0
        I = I_inrush if t < 3.0 else (3.0 if true_overload else I_work)
        w = window_rms(I, f, 2 * math.pi * f * t + phi0, nwin)
        if scheme == "new":
            nano_ema = 0.3 * w + 0.7 * nano_ema            # HOUSE_CURRENT_EMA_ALPHA
            w = nano_ema
        raw = amps_from_counts(counts_from_amps(w))
        raw = 0.0 if raw < clamp else raw
        if scheme == "new":                                # медиана по 3 пакетам
            med = sorted([hist[0], hist[1], raw])[1]
            hist = [hist[1], raw]
            raw = med
        ema = alpha * raw + (1 - alpha) * ema
        cur = ema if ema >= clamp else 0.0
        ignore = (scheme == "new") and t < 4.0             # START_CURRENT_IGNORE_MS
        if scheme == "old":
            if cur >= 1.5:                                 # emergency без маски и выдержки
                return round(t, 2)
        else:
            hard = raw >= 3.0                              # hard short >= 2x порога
            if (not ignore or hard) and cur >= 1.5:
                if em_t0 is None:
                    em_t0 = t
                if hard or t - em_t0 >= 0.25:              # EMERGENCY_CONFIRM_MS
                    return round(t, 2)
            elif cur < 1.5:
                em_t0 = None
        t += T_TELEM
    return None


if __name__ == "__main__":
    print("Сценарий 1: штатный буст 2.0 А (50 Гц, 3 c) -> 0.9 А (38 Гц), ПЕРЕГРУЗКИ НЕТ")
    ok1 = True
    for phi, tag in ((math.pi / 2, "окно на горбе"), (math.pi / 4, "окно на середине"), (0.1, "окно у нуля")):
        old = scenario("old", phi0=phi)
        new = scenario("new", phi0=phi)
        ok1 &= (new is None)
        print(f"  {tag:16s}: old trip={old} c | new trip={new}")
    print("Сценарий 2: реальная перегрузка 3.0 А после буста (защита обязана сработать)")
    old2 = scenario("old", true_overload=True)
    new2 = scenario("new", true_overload=True)
    ok2 = new2 is not None
    print(f"  old trip={old2} c | new trip={new2} c")
    print("Сценарий 3: КЗ 6 А во время буста (маскировка старта не должна спрятать)")
    old3 = scenario("old", I_inrush=6.0)
    new3 = scenario("new", I_inrush=6.0)
    ok3 = new3 is not None and new3 < 1.0
    print(f"  old trip={old3} c | new trip={new3} c")
    print("ИТОГ:", "PASS" if (ok1 and ok2 and ok3) else "FAIL")
```

## Приложение E — FALSE_ALARM_ANALYSIS.md

```markdown
# Причина ложных аварий «Дом: авария — аварийная перегрузка по току»

Ветка: `acs712-voltage-divider-replacement-aa5a5` (коммит `4979dc0` — замена делителя 0–10 В
на ACS712-30A на пине A0 Nano).

## Коротко

Авария ложная, потому что «RMS» тока дома измеряется окном **12 отсчётов АЦП ≈ 1.25 мс**
(`v1.ino:89, 204–212`) — это **1/16 периода 50 Гц**. Такое «RMS» на самом деле является
мгновенным значением тока и в зависимости от фазы даёт **от 0.16 до 1.41 × Iист**.
Пик-выбросы попадают в телеметрию (100 мс), фильтр защиты на ESP32 (EMA α=0.45,
`esp32_controller.ino:1315`) пропускает одиночный выброс почти без ослабления, а ветка
**EMERGENCY (≥1.5 А) срабатывает мгновенно, без задержки и без маскировки старта**
(`esp32_controller.ino:1857`), в то время как буст-старт длится 3 с на 50 Гц — именно там
ток наибольший (пусковой). Итог: каждый неудачный старт → «аварийная перегрузка по току».

## Цепочка ошибки (с номерами строк)

1. **Nano, `v1.ino:89`** — `HOUSE_CURRENT_AVG_SAMPLES = 12`.
   `analogRead` на AVR Nano ≈ 104 мкс (prescaler 128, 13 тактов) → окно ≈ **1.25 мс = 22.5°**
   периода 50 Гц. Среднее sin² по окну = ½ + sin(W)·cos(2φ+W)/(2W), |sinW|/(2W) = 0.487 →
   показание = √2·I·√(среднее sin²) ∈ **[0.16·I … 1.41·I]** в зависимости от фазы φ.
   Для сравнения: скважина меряется 120 отсчётами (`v1.ino:88`), а старый делитель 0–10 В
   давал вообще гладкий DC-сигнал — проблемы не существовало.
2. **Nano, `v1.ino:449–455`** — телеметрия каждые 100 мс несёт одно такое «оконное» значение
   без дополнительного сглаживания.
3. **Алиасинг/фазовая синхронизация**: 100 мс = ровно 5 периодов при 50.0 Гц. Во время
   буста (`houseCtrl::BOOST_FREQ = 50.0`) фаза окна **одинакова в каждом пакете** → если фаза
   попала на горб синусоиды, все пакеты в течение всего буста систематически завышены в ~1.4
   раза (нет усреднения по пакетам).
4. **ESP32, `esp32_controller.ino:1315`** — `PROTECTION_FILTER_ALPHA = 0.45`: один пакет с
   выбросом S сдвигает защитное значение на 45 % к S. При prev ≈ 1.0 А для пробоя порога
   1.5 А достаточно одного пакета с S ≥ (1.5 − 0.55·1.0)/0.45 ≈ **2.04 А** — пусковой ток
   буста, умноженный на 1.41, легко это даёт.
5. **ESP32, `esp32_controller.ino:1855–1864`** — ключевая логическая ошибка:
   `ignoreStartCurrent` (первые 2500 мс после старта) применяется к перегрузке 1.3 А/5 с
   (стр. 1866) и сухому ходу (стр. 1899), но **НЕ к аварийной ветке ≥1.5 А (стр. 1857)**,
   у которой к тому же нет ни выдержки времени, ни подтверждения N пакетами.
   Дополнительно: 2500 мс маскировки < 3000 мс буста (`houseCtrl:295–297`) — последние
   0.5 с буста не маскированы даже для перегрузки.
6. Результат в логах (скриншот): `запуск насоса, буст 50 Гц на 3 с` → сразу
   `авария — аварийная перегрузка по току` → `FAULT`, `Ток: 0.2 А` в останове.

## Сопутствующие симптомы того же дефекта

- **Ток 0.2 А в полном останове** (на скриншоте): шумовой пол ACS712-30A
  (~155 мкВ/√Гц · √4.8 кГц ≈ 10 мВ RMS ≈ 0.15–0.2 А) попадает в несглаженное «RMS».
  Клампы `<0.1 А` (Nano) и `NEAR_ZERO_CLAMP 0.08 А` (ESP32) его не срезают.
  Калибровка нуля выполняется только при старте (`v1.ino:548–556`) — дрейф не компенсируется.
- **Дёрганое показание тока на UI** во время работы (фаза окна плавает при f ≠ 50 Гц:
  при 38 Гц сдвиг фазы 288°/пакет → каждый 5-й пакет близок к пику).
- В `Osnova.ino:343–357` тот же класс ошибки: окно 120 отсчётов = 12.5 мс = 0.625 периода
  (±9 % на 50 Гц, ±17 % на 28 Гц) и мгновенный аварийный_trip без сглаживания
  (у скважины хотя бы есть EMA в `readCurrent()`).

## Что чинить

1. **Nano (`v1.ino`)**: окно RMS по времени, а не по 12 отсчётам — ≥ 2 периодов
   (например, набор отсчётов в течение 60 мс ≈ 300–600 шт., или детект нуля и целое число
   периодов) + EMA на самой Nano (α ≈ 0.3). Это убирает разброс 0.16…1.41 и фазовую
   синхронизацию с телеметрий.
2. **ESP32 (`esp32_controller.ino:1857`)**: распространить `ignoreStartCurrent` на аварийную
   ветку И увеличить окно до ≥ буста + запас (3500–4000 мс); аварийной ветке добавить
   подтверждение (2–3 пакета подряд или выдержку 150–250 мс).
3. **ESP32 (`esp32_controller.ino:1315`)**: уменьшить `PROTECTION_FILTER_ALPHA` до 0.15–0.25
   или вставить медианный фильтр по 3 пакетам перед EMA — одиночный выброс перестанет
   пробивать порог.
4. **Ноль/шум**: перекалибровка нуля в останове (при `vfdRun == false`, скользящим средним),
   поднять `NEAR_ZERO_CLAMP_A` до ~0.25 А, чтобы шумовой пол 0.15–0.2 А не светился как ток.
5. Долгосрочно: ACS712-30A (66 мВ/А) для рабочего тока 0.6–1.0 А — плохое отношение
   сигнал/шум; рассмотреть ACS711/трансформатор тока или аппаратный ФНЧ ~100–500 Гц
   перед АЦП.

Пункты 1–3 устраняют причину ложных срабатываний; пункт 4 убирает «0.2 А в останове».

---

# Применённые правки (ветка `fix/house-current-false-alarms`)

## `v1.ino` (Nano)
- `HOUSE_CURRENT_AVG_SAMPLES = 12` заменено на **временное окно 40 мс**
  (`HOUSE_CURRENT_WINDOW_US = 40000`, ~380 отсчётов = 2 периода 50 Гц, >1 периода 28 Гц)
  + EMA α=0.30 между окнами (`HOUSE_CURRENT_EMA_ALPHA`). Разброс 0.16…1.41 устранён.
- Добавлена **автокалибровка нуля в останове**: при `houseMode` ∈ {WAIT_WATER, READY,
  STOPPED, FAULT} дольше 5 с смещение нуля медленно подстраивается
  (`HOUSE_ZERO_ADAPT_ALPHA = 0.02`) — компенсирует дрейф, убирет «0.2 А в останове».
- Формат телеметрии (`analogAuxRaw` = RMS в отсчётах АЦП) и конвертация на ESP32 НЕ менялись.

## `esp32_controller.ino`
- `houseCtrl::START_CURRENT_IGNORE_MS`: 2500 → **4000 мс** (покрывает буст 3000 мс + запас).
- Аварийная ветка ≥1.5 А (`runProtections`): теперь **маскируется на старте** и требует
  **выдержки `EMERGENCY_CONFIRM_MS = 250 мс`**; мгновенный trip сохранён только для
  жёсткого КЗ (`EMERGENCY_START_HARD_LIMIT_MULT = 2.0`, по необработанному отсчёту
  `tm.houseCurrentRawLast`).
- `houseCurrentSense::PROTECTION_FILTER_ALPHA`: 0.45 → **0.20**; перед фильтром добавлен
  **медианный фильтр по 3 пакетам** (`tm.houseCurrentRawHist`) — одиночный выброс больше
  не пробивает порог.
- `houseCurrentSense::NEAR_ZERO_CLAMP_A`: 0.08 → **0.25** (шумовой пол ACS712-30A
  ~0.15–0.2 А не отображается как ток).
- Добавлены поля `tm.houseCurrentRawHist[2]`, `tm.houseCurrentRawLast`,
  `st.houseEmergencyStart` (сбрасывается во всех точках сброса `houseOverloadStart`).

## Проверка
- `arduino-cli compile --fqbn arduino:avr:nano` (v1.ino): OK, 10444 B flash / 491 B RAM.
- `arduino-cli compile --fqbn esp32:esp32:esp32` (esp32_controller.ino): OK, 1068144 B flash.
- Симуляция цепи измерения (окно АЦП 104 мкс, телеметрия 100 мс, буст 2.0 А/50 Гц 3 с,
  далее 0.9 А/38 Гц): **старая схема — trip на 0.1 с при большинстве фаз окна (ложно);
  новая схема — trip нет**. Контроль защит: реальная перегрузка 3.0 А после буста —
  new trip 4.3 с (маскировка старта + выдержка); КЗ 6 А на старте — new trip 0.4 с.
```
