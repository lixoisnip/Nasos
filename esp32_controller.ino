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

constexpr float PRESSURE_WARN = 1.20f;
constexpr float PRESSURE_BLOCK = 1.50f;
constexpr unsigned long PRESSURE_CHECK_DELAY = 8000UL;
constexpr unsigned long CURRENT_CHECK_DELAY = 15000UL;
constexpr float CURRENT_MIN_START = 2.9f;
constexpr int FAILED_START_LIMIT = 3;
constexpr float TARGET_MIN = 5.0f;
constexpr float MIN_PAUSE_MS = 10000.0f;
constexpr float MAX_PAUSE_MS = 90000.0f;

constexpr unsigned long START_CURRENT_IGNORE_MS = 2000UL;
constexpr unsigned long DRY_PRESSURE_START_TIMEOUT = 10000UL;
constexpr float PRESSURE_RISE_THRESHOLD = 0.1f;
constexpr unsigned long DRY_PRESSURE_WORK_TIMEOUT = 15000UL;
constexpr unsigned long PID_PERIOD_MS = 300UL;
constexpr unsigned long FREQ_STEP_PERIOD_MS = 500UL;
constexpr float INTEGRAL_LIMIT = 18.0f;

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
  float litersPerMin = 30.0f;
  float setpointBar = 1.0f;
  float houseHystOn = 0.50f;
  float houseHystOff = 1.18f;
  float houseMinFreq = 28.0f;
  float houseMaxFreq = 50.0f;
  float houseDryCurrent = 0.4f;
  float houseOverloadCurrent = 1.3f;
  float houseEmergencyCurrent = 1.5f;
  unsigned long houseDryDelayMs = 8000UL;
  unsigned long houseOverloadDelayMs = 5000UL;
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

  float e = TARGET_MIN - workedMinutes;
  float gain = fabs(e) > 2.0f ? 2.5f : 1.0f;
  st.pidInt += e * gain;
  float p = 1.5f * e * gain;
  float d = 0.2f * (e - st.pidLastE);
  st.pidLastE = e;
  st.pauseMs = constrain(st.pauseMs + p + st.pidInt + d, MIN_PAUSE_MS, MAX_PAUSE_MS);

  st.wellPauseStart = now;
  resetWellStartChecks();
  appendLog(st.logsWell, "WELL: stop " + reason);
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

void sendNanoCommand() { NanoSerial.printf("RELAY=%d;VFD_RUN=%d;VFD_FREQ=%.1f\n", st.wellRelay ? 1 : 0, st.vfdRun ? 1 : 0, st.vfdFreq); }

void runWellLogic(unsigned long now) {
  if (!tm.valid || st.wellBlocked) { st.wellRelay = false; return; }

  st.filterWarning = tm.wellPressure >= PRESSURE_WARN;
  if (tm.wellPressure >= PRESSURE_BLOCK) st.pressureBlock = true;

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
    appendLog(st.logsWell, st.wellForceMode ? "WELL: force start" : "WELL: start");
  }

  if (st.wellRelay && !st.wellForceMode && tm.levels[3]) {
    st.intention = INT_TARGET_REACHED;
    handleWellStop(now, "by L4");
  }

  if (st.wellRelay && st.wellStartChecksPending) {
    if ((now - st.wellStartTs) >= PRESSURE_CHECK_DELAY && tm.wellPressure <= 0.30f) {
      st.failedStartCount++;
      appendLog(st.logsWell, "WELL: start fail pressure");
      handleWellStop(now, "start fail");
    } else if ((now - st.wellStartTs) >= CURRENT_CHECK_DELAY && tm.wellCurrent < CURRENT_MIN_START) {
      st.failedStartCount++;
      appendLog(st.logsWell, "WELL: start fail current");
      handleWellStop(now, "start fail");
    } else if (tm.wellPressure > 0.30f && tm.wellCurrent >= CURRENT_MIN_START) {
      st.wellStartChecksPending = false;
      st.failedStartCount = 0;
      persistState(true);
    }
  }

  if (st.failedStartCount >= FAILED_START_LIMIT) {
    st.wellBlocked = st.wellAlarm = true;
    st.wellRelay = false;
    st.wellForceMode = false;
    resetWellStartChecks();
    appendLog(st.logsWell, "WELL: blocked by failed starts");
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
      appendLog(st.logsHouse, "HOUSE: start");
    }
    if (st.vfdRun && tm.housePressure >= cfg.houseHystOff) {
      st.vfdRun = false;
      st.vfdFreq = cfg.houseMinFreq;
      appendLog(st.logsHouse, "HOUSE: stop by pressure");
      return;
    }
  } else if (!st.vfdRun) {
    st.vfdRun = true;
    st.houseStartTs = now;
    st.houseDryPressureStart = now;
    st.houseInitialPressure = tm.housePressure;
  }

  if (!st.vfdRun) return;

  if (now - st.lastHousePidTs >= PID_PERIOD_MS) {
    st.lastHousePidTs = now;
    float error = cfg.setpointBar - tm.housePressure;
    st.housePidIntegral += error * (PID_PERIOD_MS / 1000.0f);
    st.housePidIntegral = constrain(st.housePidIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    st.targetFreq = cfg.houseMinFreq + 260.0f * error + 0.45f * st.housePidIntegral;
    st.targetFreq = constrain(st.targetFreq, cfg.houseMinFreq, cfg.houseMaxFreq);
  }

  if (now - st.lastFreqStepTs >= FREQ_STEP_PERIOD_MS) {
    st.lastFreqStepTs = now;
    float step = fabs(st.targetFreq - st.vfdFreq) > 8.0f ? 2.0f : 1.0f;
    if (st.targetFreq > st.vfdFreq) st.vfdFreq = min(st.vfdFreq + step, st.targetFreq);
    else st.vfdFreq = max(st.vfdFreq - step, st.targetFreq);
  }
}

void runProtections(unsigned long now) {
  if (st.wellRelay) {
    if (tm.wellCurrent >= 6.0f) { st.wellBlocked = st.wellAlarm = true; st.wellRelay = false; st.wellForceMode = false; appendLog(st.logsWell, "WELL: emergency overcurrent"); }
    if (tm.wellCurrent >= 4.3f) { if (!st.wellOverloadStart) st.wellOverloadStart = now; if (now - st.wellOverloadStart > 5000UL) { st.wellBlocked = st.wellAlarm = true; st.wellRelay = false; appendLog(st.logsWell, "WELL: overload"); } }
    else st.wellOverloadStart = 0;
    if (tm.wellCurrent < 3.3f) { if (!st.wellDryStart) st.wellDryStart = now; if (now - st.wellDryStart > 8000UL) { st.wellBlocked = st.wellAlarm = true; st.wellRelay = false; appendLog(st.logsWell, "WELL: dry run"); handleWellStop(now, "dry"); } }
    else st.wellDryStart = 0;
  }

  if (st.vfdRun) {
    if (tm.houseCurrent >= cfg.houseEmergencyCurrent) {
      st.houseBlocked = st.houseAlarm = true; st.vfdRun = false; st.houseForceMode = false;
      appendLog(st.logsHouse, "HOUSE: emergency overcurrent");
    }

    bool ignoreCurr = (now - st.houseStartTs) < START_CURRENT_IGNORE_MS;
    if (!ignoreCurr) {
      if (tm.houseCurrent >= cfg.houseOverloadCurrent) {
        if (!st.houseOverloadStart) st.houseOverloadStart = now;
        if (now - st.houseOverloadStart > cfg.houseOverloadDelayMs) {
          st.houseBlocked = st.houseAlarm = true; st.vfdRun = false; appendLog(st.logsHouse, "HOUSE: overload");
        }
      } else st.houseOverloadStart = 0;

      if (tm.houseCurrent < cfg.houseDryCurrent) {
        if (!st.houseDryStart) st.houseDryStart = now;
        if (now - st.houseDryStart > cfg.houseDryDelayMs) {
          st.houseBlocked = st.houseAlarm = true; st.vfdRun = false; appendLog(st.logsHouse, "HOUSE: dry current");
        }
      } else st.houseDryStart = 0;
    }

    if (st.houseDryPressureStart && (now - st.houseDryPressureStart) >= DRY_PRESSURE_START_TIMEOUT) {
      if (tm.housePressure < st.houseInitialPressure + PRESSURE_RISE_THRESHOLD) {
        st.houseBlocked = st.houseAlarm = true; st.vfdRun = false; appendLog(st.logsHouse, "HOUSE: dry pressure start");
      }
      st.houseDryPressureStart = 0;
    }

    if (tm.housePressure < cfg.houseHystOn) {
      if (!st.houseLowPressureStart) st.houseLowPressureStart = now;
      if (now - st.houseLowPressureStart > DRY_PRESSURE_WORK_TIMEOUT) {
        st.houseBlocked = st.houseAlarm = true; st.vfdRun = false; appendLog(st.logsHouse, "HOUSE: dry pressure work");
      }
    } else {
      st.houseLowPressureStart = 0;
    }
  }
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
  server.on("/logs_well", HTTP_GET, []() { server.send(200, "text/plain; charset=utf-8", st.logsWell); });
  server.on("/logs_house", HTTP_GET, []() { server.send(200, "text/plain; charset=utf-8", st.logsHouse); });

  server.on("/set", HTTP_POST, []() {
    if (!server.hasArg("param") || !server.hasArg("value")) return server.send(400, "text/plain", "Missing param/value");
    String p = server.arg("param");
    float v = server.arg("value").toFloat();
    if (p == "SETPOINT_BAR") cfg.setpointBar = constrain(v, 0.0f, 2.0f);
    server.send(200, "text/plain", "OK");
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
  appendLog(st.logsWell, "System start");
  appendLog(st.logsHouse, "System start");
}

void loop() {
  esp_task_wdt_reset();
  unsigned long now = millis();
  readNanoUart();
  runWellLogic(now);
  runHouseLogic(now);
  runProtections(now);
  sendNanoCommand();
  server.handleClient();
  delay(5);
}
