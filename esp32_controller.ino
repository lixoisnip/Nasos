// =====================================================
// ESP32 main controller: logic/protections/web + persistence
// =====================================================

#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <math.h>

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* AP_SSID = "Nasos-ESP32";
const char* AP_PASS = "12345678";

HardwareSerial NanoSerial(2);
constexpr int NANO_RX_PIN = 16;
constexpr int NANO_TX_PIN = 17;
constexpr uint32_t NANO_BAUD = 38400;

constexpr unsigned long CURRENT_CHECK_DELAY = 15000UL;
constexpr int LEVEL_FILTER_MS = 2000;
constexpr int INIT_DELAY_MS = 6000;
constexpr int THRESH = 700;

enum WellIntention : uint8_t { INT_TARGET_REACHED = 0, INT_FILL_TO_L4 = 1 };

struct Telemetry {
  unsigned long ts = 0;
  float wellCurrent = 0;
  float wellPressure = 0;
  float houseCurrent = 0;
  float housePressureRaw = 0;
  float housePressure = 0;
  bool levels[4] = {false, false, false, false};
  bool vfdRunFeedback = false;
  float vfdFreqFeedback = 0;
  bool valid = false;
  float pressureFilterBuf[6] = {0};
  uint8_t pressureFilterIdx = 0;
  bool pressureFilterInit = false;
} tm;

struct Settings {
  float targetMin = 5.0f;
  float litersPerMin = 30.0f;
  float minPauseMin = 10.0f;
  float maxPauseMin = 90.0f;
  float wellDryCurrent = 3.3f;
  float wellOverloadCurrent = 4.3f;
  float wellEmergencyCurrent = 6.0f;
  float currentMinStart = 2.9f;
  float pressureMinOk = 0.30f;
  float pressureWarning = 1.20f;
  float pressureBlock = 1.50f;
  unsigned long wellOverloadDelayMs = 5000UL;
  unsigned long wellDryDelayMs = 8000UL;
  unsigned long pressureCheckDelay = 8000UL;
  int maxFailedStarts = 3;

  float setpointBar = 1.0f;
  float houseHystOn = 0.50f;
  float houseHystOff = 1.18f;
  float houseMinFreq = 28.0f;
  float houseMaxFreq = 50.0f;
  float shutdownFreq = 34.0f;
  float houseCurrentNormalMin = 0.6f;
  float houseCurrentNormalMax = 1.0f;
  float houseDryCurrent = 0.4f;
  float houseOverloadCurrent = 1.3f;
  float houseEmergencyCurrent = 1.5f;
  unsigned long houseDryDelayMs = 8000UL;
  unsigned long houseOverloadDelayMs = 5000UL;
  unsigned long startCurrentIgnoreMs = 2000UL;
  unsigned long dryPressureStartTimeout = 10000UL;
  unsigned long dryPressureWorkTimeout = 15000UL;
  float pressureRiseThreshold = 0.1f;
  float kp = 260.0f;
  float ki = 0.45f;
  float integralLimit = 18.0f;
  unsigned long pidPeriod = 300UL;
  unsigned long freqStepDelay = 500UL;
} cfg;

struct Controller {
  bool wellRelay = false;
  bool vfdRun = false;
  float vfdFreq = 28.0f;

  bool wellBlocked = false;
  bool wellAlarm = false;
  bool houseBlocked = false;
  bool houseAlarm = false;
  bool filterWarning = false;
  bool pressureBlock = false;

  bool wellForceMode = false;
  bool houseForceMode = false;

  unsigned long wellRunStart = 0;
  unsigned long wellPauseStart = 0;
  unsigned long wellDryStart = 0;
  unsigned long wellOverloadStart = 0;
  unsigned long houseDryStart = 0;
  unsigned long houseOverloadStart = 0;
  unsigned long houseStartTs = 0;
  unsigned long houseDryPressureStart = 0;
  unsigned long houseLowPressureStart = 0;
  unsigned long lastHousePidTs = 0;
  unsigned long lastFreqStepTs = 0;
  unsigned long lastNanoCmdTs = 0;

  unsigned long wellStartTs = 0;
  bool wellStartChecksPending = false;
  int failedStartCount = 0;

  float housePidIntegral = 0;
  float targetFreq = 28.0f;
  float houseInitialPressure = 0.0f;

  unsigned long lastWorkSec = 0;
  float pauseMs = 30000;
  float totalLiters = 0;
  WellIntention intention = INT_TARGET_REACHED;
  float pidInt = 0;
  float pidLastE = 0;

  unsigned long lastPersistLitersTs = 0;
  float lastPersistLitersVal = 0;

  float volumeHistory[20] = {0};
  float workHistory[20] = {0};

  String logsWell;
  String logsHouse;
} st;

WebServer server(80);
Preferences prefs;

void appendLog(String& dst, const String& msg) { dst += msg + "\n"; if (dst.length() > 5000) dst.remove(0, dst.length() - 5000); }
void pushHistory(float* arr, float value) { for (int i = 0; i < 19; i++) arr[i] = arr[i + 1]; arr[19] = value; }

uint8_t xorChecksum(const String &s) { uint8_t c = 0; for (size_t i = 0; i < s.length(); i++) c ^= (uint8_t)s[i]; return c; }

void persistState(bool force = false) {
  unsigned long now = millis();
  bool litersDue = force || (now - st.lastPersistLitersTs >= 600000UL) || (fabs(st.totalLiters - st.lastPersistLitersVal) >= 50.0f);
  if (litersDue) {
    prefs.putFloat("totalLit", st.totalLiters);
    st.lastPersistLitersTs = now;
    st.lastPersistLitersVal = st.totalLiters;
  }
  if (force) {
    prefs.putFloat("pauseMs", st.pauseMs);
    prefs.putInt("failCnt", st.failedStartCount);
    prefs.putUChar("intent", (uint8_t)st.intention);
    prefs.putFloat("pidInt", st.pidInt);
    prefs.putFloat("pidLastE", st.pidLastE);
  }
}

void loadState() {
  st.totalLiters = prefs.getFloat("totalLit", 0.0f);
  st.pauseMs = prefs.getFloat("pauseMs", 30000.0f);
  st.failedStartCount = prefs.getInt("failCnt", 0);
  st.intention = (WellIntention)prefs.getUChar("intent", INT_TARGET_REACHED);
  st.pidInt = prefs.getFloat("pidInt", 0.0f);
  st.pidLastE = prefs.getFloat("pidLastE", 0.0f);
  st.lastPersistLitersVal = st.totalLiters;
  st.lastPersistLitersTs = millis();
}

void resetWellStartChecks() {
  st.wellStartTs = 0;
  st.wellStartChecksPending = false;
}

void handleWellStop(unsigned long now, const String &reason) {
  if (!st.wellRelay) return;
  st.wellRelay = false;
  st.lastWorkSec = (now - st.wellRunStart) / 1000UL;
  float workedMinutes = st.lastWorkSec / 60.0f;
  float liters = st.lastWorkSec * (cfg.litersPerMin / 60.0f);
  st.totalLiters += liters;
  pushHistory(st.volumeHistory, liters);
  pushHistory(st.workHistory, st.lastWorkSec);

  float e = cfg.targetMin - workedMinutes;
  float gain = fabs(e) > 2.0f ? 2.5f : 1.0f;
  st.pidInt += e * gain;
  float p = 1.5f * e * gain;
  float d = 0.2f * (e - st.pidLastE);
  st.pidLastE = e;
  st.pauseMs = constrain(st.pauseMs + p + st.pidInt + d, (cfg.minPauseMin * 60000.0f), (cfg.maxPauseMin * 60000.0f));

  st.wellPauseStart = now;
  resetWellStartChecks();
  appendLog(st.logsWell, "СКВАЖИННЫЙ: остановка " + reason);
  persistState(true);
}

void parseNanoLine(const String& line) {
  if (!line.startsWith("TEL,")) return;
  int starPos = line.lastIndexOf(",*");
  if (starPos < 0 || starPos + 4 > (int)line.length()) return;
  String payload = line.substring(0, starPos);
  String cHex = line.substring(starPos + 2);
  uint8_t got = (uint8_t)strtoul(cHex.c_str(), nullptr, 16);
  if (xorChecksum(payload) != got) return;

  float vals[11] = {0};
  int idx = 0;
  int start = 4;
  while (idx < 11 && start < (int)payload.length()) {
    int comma = payload.indexOf(',', start);
    if (comma < 0) comma = payload.length();
    vals[idx++] = payload.substring(start, comma).toFloat();
    start = comma + 1;
  }
  if (idx < 11) return;

  tm.ts = (unsigned long)vals[0];
  tm.wellCurrent = vals[1];
  tm.wellPressure = vals[2];
  tm.houseCurrent = vals[3];
  tm.housePressureRaw = vals[4];
  if (!tm.pressureFilterInit) {
    for (uint8_t i = 0; i < 6; i++) tm.pressureFilterBuf[i] = tm.housePressureRaw;
    tm.pressureFilterInit = true;
  }
  tm.pressureFilterBuf[tm.pressureFilterIdx] = tm.housePressureRaw;
  tm.pressureFilterIdx = (tm.pressureFilterIdx + 1) % 6;
  float sum = 0;
  for (uint8_t i = 0; i < 6; i++) sum += tm.pressureFilterBuf[i];
  tm.housePressure = sum / 6.0f;

  tm.levels[0] = vals[5] > 0.5f;
  tm.levels[1] = vals[6] > 0.5f;
  tm.levels[2] = vals[7] > 0.5f;
  tm.levels[3] = vals[8] > 0.5f;
  tm.vfdRunFeedback = vals[9] > 0.5f;
  tm.vfdFreqFeedback = vals[10];
  tm.valid = true;
}

void readNanoUart() {
  static String line;
  while (NanoSerial.available()) {
    char c = (char)NanoSerial.read();
    if (c == '\n') { parseNanoLine(line); line = ""; }
    else if (c != '\r') line += c;
  }
}

void sendNanoCommand() {
  NanoSerial.printf(
    "RELAY=%d;VFD_RUN=%d;VFD_FREQ=%.1f;WB=%d;HB=%d;WA=%d;HA=%d;WF=%d;HF=%d;FW=%d;PB=%d;FSC=%d;PMS=%.0f;TL=%.1f;SP=%.2f\n",
    st.wellRelay ? 1 : 0,
    st.vfdRun ? 1 : 0,
    st.vfdFreq,
    st.wellBlocked ? 1 : 0,
    st.houseBlocked ? 1 : 0,
    st.wellAlarm ? 1 : 0,
    st.houseAlarm ? 1 : 0,
    st.wellForceMode ? 1 : 0,
    st.houseForceMode ? 1 : 0,
    st.filterWarning ? 1 : 0,
    st.pressureBlock ? 1 : 0,
    st.failedStartCount,
    st.pauseMs,
    st.totalLiters,
    cfg.setpointBar
  );
}

void runWellLogic(unsigned long now) {
  if (!tm.valid || st.wellBlocked) { st.wellRelay = false; return; }

  st.filterWarning = tm.wellPressure >= cfg.pressureWarning;
  if (tm.wellPressure >= cfg.pressureBlock) st.pressureBlock = true;

  if (tm.levels[3]) st.intention = INT_TARGET_REACHED;
  else if (!tm.levels[1]) st.intention = INT_FILL_TO_L4;

  bool levelNeed = false;
  if (!tm.levels[3]) {
    if (st.intention == INT_FILL_TO_L4) levelNeed = true;
    else levelNeed = (!tm.levels[1] || !tm.levels[2]);
  }
  bool needPump = st.wellForceMode || levelNeed;

  if (needPump && !st.wellRelay && !st.pressureBlock && (now - st.wellPauseStart > (unsigned long)st.pauseMs)) {
    st.wellRelay = true;
    st.wellRunStart = now;
    st.wellStartTs = now;
    st.wellStartChecksPending = true;
    appendLog(st.logsWell, st.wellForceMode ? "Скважинный: принудительный старт" : "Скважинный: старт");
  }

  if (st.wellRelay && !st.wellForceMode && tm.levels[3]) {
    st.intention = INT_TARGET_REACHED;
    handleWellStop(now, "по уровню L4");
  }

  if (st.wellRelay && st.wellStartChecksPending) {
    if ((now - st.wellStartTs) >= cfg.pressureCheckDelay && tm.wellPressure <= cfg.pressureMinOk) {
      st.failedStartCount++;
      appendLog(st.logsWell, "Скважинный: неудачный старт (давление не выросло)");
      handleWellStop(now, "неудачный пуск");
    } else if ((now - st.wellStartTs) >= CURRENT_CHECK_DELAY && tm.wellCurrent < cfg.currentMinStart) {
      st.failedStartCount++;
      appendLog(st.logsWell, "Скважинный: неудачный старт (низкий ток)");
      handleWellStop(now, "неудачный пуск");
    } else if (tm.wellPressure > cfg.pressureMinOk && tm.wellCurrent >= cfg.currentMinStart) {
      st.wellStartChecksPending = false;
      st.failedStartCount = 0;
      persistState(true);
    }
  }

  if (st.failedStartCount >= cfg.maxFailedStarts) {
    st.wellBlocked = st.wellAlarm = true;
    st.wellRelay = false;
    st.wellForceMode = false;
    resetWellStartChecks();
    appendLog(st.logsWell, "Скважинный: блокировка по числу неудачных пусков");
    persistState(true);
  }

  persistState();
}

void runHouseLogic(unsigned long now) {
  if (!tm.valid || st.houseBlocked) { st.vfdRun = false; st.vfdFreq = cfg.houseMinFreq; return; }

  bool l1 = tm.levels[0], l2 = tm.levels[1];
  if (!st.houseForceMode) {
    if (!l1) {
      st.vfdRun = false;
      st.vfdFreq = cfg.houseMinFreq;
      return;
    }
    if (!st.vfdRun && !l2) {
      st.vfdFreq = cfg.houseMinFreq;
      return;
    }

    if (!st.vfdRun && tm.housePressure <= cfg.houseHystOn) {
      st.vfdRun = true;
      st.houseStartTs = now;
      st.houseDryPressureStart = now;
      st.houseInitialPressure = tm.housePressure;
      st.houseLowPressureStart = 0;
      appendLog(st.logsHouse, "Домашний: старт");
    }
    if (st.vfdRun && tm.housePressure >= cfg.houseHystOff) {
      st.vfdRun = false;
      st.vfdFreq = cfg.houseMinFreq;
      appendLog(st.logsHouse, "Домашний: остановка по давлению");
      return;
    }
  } else if (!st.vfdRun) {
    st.vfdRun = true;
    st.houseStartTs = now;
    st.houseDryPressureStart = now;
    st.houseInitialPressure = tm.housePressure;
  }

  if (!st.vfdRun) return;

  if (now - st.lastHousePidTs >= cfg.pidPeriod) {
    st.lastHousePidTs = now;
    float error = cfg.setpointBar - tm.housePressure;
    st.housePidIntegral += error * (cfg.pidPeriod / 1000.0f);
    st.housePidIntegral = constrain(st.housePidIntegral, -cfg.integralLimit, cfg.integralLimit);
    st.targetFreq = cfg.houseMinFreq + cfg.kp * error + cfg.ki * st.housePidIntegral;
    st.targetFreq = constrain(st.targetFreq, cfg.houseMinFreq, cfg.houseMaxFreq);
  }

  if (now - st.lastFreqStepTs >= cfg.freqStepDelay) {
    st.lastFreqStepTs = now;
    float step = fabs(st.targetFreq - st.vfdFreq) > 8.0f ? 2.0f : 1.0f;
    if (st.targetFreq > st.vfdFreq) st.vfdFreq = min(st.vfdFreq + step, st.targetFreq);
    else st.vfdFreq = max(st.vfdFreq - step, st.targetFreq);
  }
}

void runProtections(unsigned long now) {
  if (st.wellRelay) {
    if (tm.wellCurrent >= cfg.wellEmergencyCurrent) { st.wellBlocked = st.wellAlarm = true; st.wellRelay = false; st.wellForceMode = false; appendLog(st.logsWell, "Скважинный: авария по току"); }
    if (tm.wellCurrent >= cfg.wellOverloadCurrent) { if (!st.wellOverloadStart) st.wellOverloadStart = now; if (now - st.wellOverloadStart > cfg.wellOverloadDelayMs) { st.wellBlocked = st.wellAlarm = true; st.wellRelay = false; appendLog(st.logsWell, "Скважинный: перегрузка по току"); } }
    else st.wellOverloadStart = 0;
    if (tm.wellCurrent < cfg.wellDryCurrent) { if (!st.wellDryStart) st.wellDryStart = now; if (now - st.wellDryStart > cfg.wellDryDelayMs) { st.wellBlocked = st.wellAlarm = true; st.wellRelay = false; appendLog(st.logsWell, "Скважинный: сухой ход"); handleWellStop(now, "сухой ход"); } }
    else st.wellDryStart = 0;
  }

  if (st.vfdRun) {
    if (tm.houseCurrent >= cfg.houseEmergencyCurrent) {
      st.houseBlocked = st.houseAlarm = true; st.vfdRun = false; st.houseForceMode = false;
      appendLog(st.logsHouse, "Домашний: авария по току");
    }

    bool ignoreCurr = (now - st.houseStartTs) < cfg.startCurrentIgnoreMs;
    if (!ignoreCurr) {
      if (tm.houseCurrent >= cfg.houseOverloadCurrent) {
        if (!st.houseOverloadStart) st.houseOverloadStart = now;
        if (now - st.houseOverloadStart > cfg.houseOverloadDelayMs) {
          st.houseBlocked = st.houseAlarm = true; st.vfdRun = false; appendLog(st.logsHouse, "Домашний: перегрузка по току");
        }
      } else st.houseOverloadStart = 0;

      if (tm.houseCurrent < cfg.houseDryCurrent) {
        if (!st.houseDryStart) st.houseDryStart = now;
        if (now - st.houseDryStart > cfg.houseDryDelayMs) {
          st.houseBlocked = st.houseAlarm = true; st.vfdRun = false; appendLog(st.logsHouse, "Домашний: сухой ход по току");
        }
      } else st.houseDryStart = 0;
    }

    if (st.houseDryPressureStart && (now - st.houseDryPressureStart) >= cfg.dryPressureStartTimeout) {
      if (tm.housePressure < st.houseInitialPressure + cfg.pressureRiseThreshold) {
        st.houseBlocked = st.houseAlarm = true; st.vfdRun = false; appendLog(st.logsHouse, "Домашний: сухой ход (нет роста давления при старте)");
      }
      st.houseDryPressureStart = 0;
    }

    if (tm.housePressure < cfg.houseHystOn) {
      if (!st.houseLowPressureStart) st.houseLowPressureStart = now;
      if (now - st.houseLowPressureStart > cfg.dryPressureWorkTimeout) {
        st.houseBlocked = st.houseAlarm = true; st.vfdRun = false; appendLog(st.logsHouse, "Домашний: сухой ход (низкое давление в работе)");
      }
    } else {
      st.houseLowPressureStart = 0;
    }
  }
}


void addSetting(JsonObject obj, const char* key, float value) { obj[key] = value; }
void addSetting(JsonObject obj, const char* key, unsigned long value) { obj[key] = value; }
void addSetting(JsonObject obj, const char* key, int value) { obj[key] = value; }

String buildJsonSettings() {
  StaticJsonDocument<4096> doc;
  JsonObject c = doc.to<JsonObject>();
  addSetting(c, "TARGET_MIN", cfg.targetMin);
  addSetting(c, "L_PER_MIN", cfg.litersPerMin);
  addSetting(c, "MIN_PAUSE", cfg.minPauseMin);
  addSetting(c, "MAX_PAUSE", cfg.maxPauseMin);
  addSetting(c, "CURRENT_DRY_WELL", cfg.wellDryCurrent);
  addSetting(c, "CURRENT_OVERLOAD_WELL", cfg.wellOverloadCurrent);
  addSetting(c, "CURRENT_EMERGENCY_WELL", cfg.wellEmergencyCurrent);
  addSetting(c, "CURRENT_MIN_START", cfg.currentMinStart);
  addSetting(c, "PRESSURE_MIN_OK", cfg.pressureMinOk);
  addSetting(c, "PRESSURE_WARNING", cfg.pressureWarning);
  addSetting(c, "PRESSURE_BLOCK", cfg.pressureBlock);
  addSetting(c, "OVERLOAD_DELAY_MS_WELL", cfg.wellOverloadDelayMs);
  addSetting(c, "DRY_DELAY_MS_WELL", cfg.wellDryDelayMs);
  addSetting(c, "PRESSURE_CHECK_DELAY", cfg.pressureCheckDelay);
  addSetting(c, "MAX_FAILED_STARTS", cfg.maxFailedStarts);
  addSetting(c, "SETPOINT_BAR", cfg.setpointBar);
  addSetting(c, "HYST_ON", cfg.houseHystOn);
  addSetting(c, "HYST_OFF", cfg.houseHystOff);
  addSetting(c, "MIN_FREQ", cfg.houseMinFreq);
  addSetting(c, "MAX_FREQ", cfg.houseMaxFreq);
  addSetting(c, "SHUTDOWN_FREQ", cfg.shutdownFreq);
  addSetting(c, "CURRENT_NORMAL_MIN", cfg.houseCurrentNormalMin);
  addSetting(c, "CURRENT_NORMAL_MAX", cfg.houseCurrentNormalMax);
  addSetting(c, "CURRENT_DRY_HOUSE", cfg.houseDryCurrent);
  addSetting(c, "CURRENT_OVERLOAD_HOUSE", cfg.houseOverloadCurrent);
  addSetting(c, "CURRENT_EMERGENCY_HOUSE", cfg.houseEmergencyCurrent);
  addSetting(c, "OVERLOAD_DELAY_MS_HOUSE", cfg.houseOverloadDelayMs);
  addSetting(c, "DRY_DELAY_MS_HOUSE", cfg.houseDryDelayMs);
  addSetting(c, "START_CURRENT_IGNORE_MS", cfg.startCurrentIgnoreMs);
  addSetting(c, "DRY_PRESSURE_START_TIMEOUT", cfg.dryPressureStartTimeout);
  addSetting(c, "DRY_PRESSURE_WORK_TIMEOUT", cfg.dryPressureWorkTimeout);
  addSetting(c, "PRESSURE_RISE_THRESHOLD", cfg.pressureRiseThreshold);
  addSetting(c, "Kp", cfg.kp);
  addSetting(c, "Ki", cfg.ki);
  addSetting(c, "INTEGRAL_LIMIT", cfg.integralLimit);
  addSetting(c, "PID_PERIOD", cfg.pidPeriod);
  addSetting(c, "FREQ_STEP_DELAY", cfg.freqStepDelay);
  addSetting(c, "LEVEL_FILTER_MS", LEVEL_FILTER_MS);
  addSetting(c, "INIT_DELAY_MS", INIT_DELAY_MS);
  addSetting(c, "THRESH", THRESH);
  String out; serializeJson(doc, out); return out;
}

String buildJsonState() {
  StaticJsonDocument<3072> doc;
  doc["well_current"] = tm.wellCurrent;
  doc["well_pressure"] = tm.wellPressure;
  doc["house_current"] = tm.houseCurrent;
  doc["house_pressure"] = tm.housePressure;
  doc["last_work_sec"] = st.lastWorkSec;
  doc["pause_ms"] = st.pauseMs;
  doc["total_liters"] = st.totalLiters;
  doc["well_alarm"] = st.wellAlarm;
  doc["house_alarm"] = st.houseAlarm;
  doc["well_force"] = st.wellForceMode;
  doc["house_force"] = st.houseForceMode;
  doc["well_blocked"] = st.wellBlocked;
  doc["house_blocked"] = st.houseBlocked;
  doc["failed_start_count"] = st.failedStartCount;
  doc["filter_warning"] = st.filterWarning;
  doc["pressure_block"] = st.pressureBlock;
  JsonArray lv = doc.createNestedArray("levels");
  for (int i = 0; i < 4; i++) lv.add(tm.levels[i]);
  String out; serializeJson(doc, out); return out;
}

void initWeb() {
  LittleFS.begin(true);
  server.on("/", HTTP_GET, []() { File file = LittleFS.open("/index.html", "r"); if (!file) return server.send(500, "text/plain", "index.html missing"); server.streamFile(file, "text/html; charset=utf-8"); file.close(); });
  server.on("/state", HTTP_GET, []() { server.send(200, "application/json", buildJsonState()); });
  server.on("/settings", HTTP_GET, []() { server.send(200, "application/json", buildJsonSettings()); });
  server.on("/logs_well", HTTP_GET, []() { server.send(200, "text/plain; charset=utf-8", st.logsWell); });
  server.on("/logs_house", HTTP_GET, []() { server.send(200, "text/plain; charset=utf-8", st.logsHouse); });
  server.on("/clear_logs_well", HTTP_POST, []() { st.logsWell = ""; server.send(200, "text/plain", "OK"); });
  server.on("/clear_logs_house", HTTP_POST, []() { st.logsHouse = ""; server.send(200, "text/plain", "OK"); });

  server.on("/set", HTTP_POST, []() {
    if (!server.hasArg("param") || !server.hasArg("value")) return server.send(400, "text/plain", "Missing param/value");
    String p = server.arg("param");
    float v = server.arg("value").toFloat();
    bool ok = true;
    if (p == "TARGET_MIN") cfg.targetMin = v;
    else if (p == "L_PER_MIN") cfg.litersPerMin = v;
    else if (p == "MIN_PAUSE") cfg.minPauseMin = v;
    else if (p == "MAX_PAUSE") cfg.maxPauseMin = v;
    else if (p == "CURRENT_DRY_WELL") cfg.wellDryCurrent = v;
    else if (p == "CURRENT_OVERLOAD_WELL") cfg.wellOverloadCurrent = v;
    else if (p == "CURRENT_EMERGENCY_WELL") cfg.wellEmergencyCurrent = v;
    else if (p == "CURRENT_MIN_START") cfg.currentMinStart = v;
    else if (p == "PRESSURE_MIN_OK") cfg.pressureMinOk = v;
    else if (p == "PRESSURE_WARNING") cfg.pressureWarning = v;
    else if (p == "PRESSURE_BLOCK") cfg.pressureBlock = v;
    else if (p == "OVERLOAD_DELAY_MS_WELL") cfg.wellOverloadDelayMs = (unsigned long)v;
    else if (p == "DRY_DELAY_MS_WELL") cfg.wellDryDelayMs = (unsigned long)v;
    else if (p == "PRESSURE_CHECK_DELAY") cfg.pressureCheckDelay = (unsigned long)v;
    else if (p == "MAX_FAILED_STARTS") cfg.maxFailedStarts = (int)v;
    else if (p == "SETPOINT_BAR") cfg.setpointBar = v;
    else if (p == "HYST_ON") cfg.houseHystOn = v;
    else if (p == "HYST_OFF") cfg.houseHystOff = v;
    else if (p == "MIN_FREQ") cfg.houseMinFreq = v;
    else if (p == "MAX_FREQ") cfg.houseMaxFreq = v;
    else if (p == "SHUTDOWN_FREQ") cfg.shutdownFreq = v;
    else if (p == "CURRENT_NORMAL_MIN") cfg.houseCurrentNormalMin = v;
    else if (p == "CURRENT_NORMAL_MAX") cfg.houseCurrentNormalMax = v;
    else if (p == "CURRENT_DRY_HOUSE") cfg.houseDryCurrent = v;
    else if (p == "CURRENT_OVERLOAD_HOUSE") cfg.houseOverloadCurrent = v;
    else if (p == "CURRENT_EMERGENCY_HOUSE") cfg.houseEmergencyCurrent = v;
    else if (p == "OVERLOAD_DELAY_MS_HOUSE") cfg.houseOverloadDelayMs = (unsigned long)v;
    else if (p == "DRY_DELAY_MS_HOUSE") cfg.houseDryDelayMs = (unsigned long)v;
    else if (p == "START_CURRENT_IGNORE_MS") cfg.startCurrentIgnoreMs = (unsigned long)v;
    else if (p == "DRY_PRESSURE_START_TIMEOUT") cfg.dryPressureStartTimeout = (unsigned long)v;
    else if (p == "DRY_PRESSURE_WORK_TIMEOUT") cfg.dryPressureWorkTimeout = (unsigned long)v;
    else if (p == "PRESSURE_RISE_THRESHOLD") cfg.pressureRiseThreshold = v;
    else if (p == "Kp") cfg.kp = v;
    else if (p == "Ki") cfg.ki = v;
    else if (p == "INTEGRAL_LIMIT") cfg.integralLimit = v;
    else if (p == "PID_PERIOD") cfg.pidPeriod = (unsigned long)v;
    else if (p == "FREQ_STEP_DELAY") cfg.freqStepDelay = (unsigned long)v;
    else ok = false;
    server.send(ok ? 200 : 400, "text/plain", ok ? "OK" : "Unknown param");
  });

  server.on("/action", HTTP_POST, []() {
    String pump = server.arg("pump");
    String cmd = server.arg("cmd");
    if (pump == "well") {
      if (cmd == "reset_alarm") {
        st.wellBlocked = st.wellAlarm = false; st.failedStartCount = 0; st.pressureBlock = false; st.wellDryStart = st.wellOverloadStart = 0; resetWellStartChecks(); persistState(true);
      } else if (cmd == "force_on") st.wellForceMode = true;
      else if (cmd == "force_off") st.wellForceMode = false;
    } else if (pump == "house") {
      if (cmd == "reset_alarm") {
        st.houseBlocked = st.houseAlarm = false; st.houseDryStart = st.houseOverloadStart = 0; st.houseLowPressureStart = 0;
      } else if (cmd == "force_on") st.houseForceMode = true;
      else if (cmd == "force_off") st.houseForceMode = false;
    }
    server.send(200, "text/plain", "OK");
  });
  server.begin();
}

void initWiFi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000UL) delay(300);
}

void setup() {
  Serial.begin(115200);
  NanoSerial.begin(NANO_BAUD, SERIAL_8N1, NANO_RX_PIN, NANO_TX_PIN);
  esp_task_wdt_init(5, true);
  esp_task_wdt_add(NULL);

  prefs.begin("nasos", false);
  loadState();

  initWiFi();
  initWeb();
  appendLog(st.logsWell, "Система запущена");
  appendLog(st.logsHouse, "Система запущена");
}

void loop() {
  esp_task_wdt_reset();
  unsigned long now = millis();
  readNanoUart();
  runWellLogic(now);
  runHouseLogic(now);
  runProtections(now);
  if (now - st.lastNanoCmdTs >= 100UL) {
    st.lastNanoCmdTs = now;
    sendNanoCommand();
  }
  server.handleClient();
  delay(5);
}
