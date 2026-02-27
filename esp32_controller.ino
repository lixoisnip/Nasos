// =====================================================
// ESP32 main controller: all pump logic/protections + web UI
// Works with Arduino Nano I/O bridge over UART2.
// =====================================================

#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// -------- Wi-Fi settings --------
// 1) STA mode: ESP32 connects to your router.
// 2) AP mode: ESP32 always raises its own Wi-Fi for direct connection.
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* AP_SSID = "Nasos-ESP32";
const char* AP_PASS = "12345678";  // min 8 chars for WPA2

struct WellConfig {
  float dryCurrent;
  float overloadCurrent;
  float emergencyCurrent;
  unsigned long dryDelayMs;
  unsigned long overloadDelayMs;
  float targetMinutes;
  float litersPerMin;
};

struct HouseConfig {
  float setpointBar;
  float hystOn;
  float hystOff;
  float minFreq;
  float maxFreq;
  float dryCurrent;
  float overloadCurrent;
  float emergencyCurrent;
  unsigned long dryDelayMs;
  unsigned long overloadDelayMs;
};

struct CommonConfig {
  float pauseMinMs;
  float pauseMaxMs;
};

struct NetworkConfig {
  String wifiSsid;
  String wifiPass;
  String apSsid;
  String apPass;
};

namespace defaults {
const WellConfig well = {
  3.3f,
  4.3f,
  6.0f,
  8000UL,
  5000UL,
  5.0f,
  30.0f
};

const HouseConfig house = {
  1.0f,
  0.50f,
  1.18f,
  28.0f,
  50.0f,
  0.4f,
  1.3f,
  1.5f,
  8000UL,
  5000UL
};

const CommonConfig common = {
  10000.0f,
  90000.0f
};

const NetworkConfig network = {
  WIFI_SSID,
  WIFI_PASS,
  AP_SSID,
  AP_PASS
};
}

namespace wellCtrl {
constexpr float CURRENT_MIN_START = 2.9f;
constexpr float PRESSURE_MIN_OK = 0.30f;
constexpr float PRESSURE_WARNING = 1.20f;
constexpr float PRESSURE_BLOCK = 1.50f;
constexpr unsigned long CURRENT_CHECK_DELAY = 15000UL;
constexpr unsigned long PRESSURE_CHECK_DELAY = 8000UL;
constexpr int MAX_FAILED_STARTS = 3;

constexpr float PID_KP = 1.5f;
constexpr float PID_KI = 0.1f;
constexpr float PID_KD = 0.2f;
constexpr float PID_BOOST_ERROR = 2.0f;
constexpr float PID_BOOST_FACTOR = 2.5f;
constexpr float PID_INT_MIN = -40.0f;
constexpr float PID_INT_MAX = 40.0f;
constexpr const char* PREF_NAMESPACE = "well_state";
constexpr unsigned long SAVE_INTERVAL_MS = 10UL * 60UL * 1000UL;
constexpr float SAVE_LITERS_DELTA = 50.0f;
}

enum class PumpIntention : uint8_t {
  UNKNOWN,
  TARGET_REACHED,
  PUMPING_TO_L4
};

enum class WellMode : uint8_t {
  WAIT,
  STARTING,
  RUN,
  FAIL
};

// -------- UART to Nano --------
HardwareSerial NanoSerial(2);
constexpr int NANO_RX_PIN = 16;
constexpr int NANO_TX_PIN = 17;
constexpr uint32_t NANO_BAUD = 38400;

struct Telemetry {
  unsigned long ts = 0;
  float wellCurrent = 0;
  float wellPressure = 0;
  float houseCurrent = 0;
  float housePressure = 0;
  bool levels[4] = {false, false, false, false};
  bool vfdRunFeedback = false;
  float vfdFreqFeedback = 0;
  bool valid = false;
} tm;

struct Settings {
  WellConfig well;
  HouseConfig house;
  CommonConfig common;
  NetworkConfig network;
} cfg;

struct Controller {
  bool wellRelay = false;
  bool vfdRun = false;
  float vfdFreq = 28.0f;

  bool wellBlocked = false;
  bool wellAlarm = false;
  bool filterWarning = false;
  bool pressureBlock = false;
  bool houseBlocked = false;
  bool houseAlarm = false;

  bool wellForceMode = false;
  bool houseForceMode = false;

  unsigned long wellRunStart = 0;
  unsigned long wellPauseStart = 0;
  unsigned long wellStartAttempt = 0;
  unsigned long wellDryStart = 0;
  unsigned long wellOverloadStart = 0;
  unsigned long houseDryStart = 0;
  unsigned long houseOverloadStart = 0;

  unsigned long lastWorkSec = 0;
  float pauseMs = 0;
  float totalLiters = 0;
  int failedStartCount = 0;

  bool needPump = false;
  bool targetOk = false;
  PumpIntention intention = PumpIntention::UNKNOWN;
  WellMode wellMode = WellMode::WAIT;

  bool startCurrentOk = false;
  bool startPressureOk = false;
  float pidInt = 0;
  float pidLastE = 0;

  float volumeHistory[20] = {0};
  float workHistory[20] = {0};

  String logsWell;
  String logsHouse;
} st;

Preferences wellPrefs;
unsigned long wellStateLastSaveMs = 0;
float wellStateLastSaveLiters = 0;
float persistedPauseMs = 0;
int persistedFailedStartCount = 0;
PumpIntention persistedIntention = PumpIntention::UNKNOWN;

bool floatChanged(float a, float b, float eps = 0.01f) {
  return fabs(a - b) > eps;
}

void saveWellState(bool force = false) {
  bool changed = force;

  if (floatChanged(st.pauseMs, persistedPauseMs)) {
    persistedPauseMs = st.pauseMs;
    changed = true;
  }

  if (st.failedStartCount != persistedFailedStartCount) {
    persistedFailedStartCount = st.failedStartCount;
    changed = true;
  }

  if (st.intention != persistedIntention) {
    persistedIntention = st.intention;
    changed = true;
  }

  bool litersDeltaReached = fabs(st.totalLiters - wellStateLastSaveLiters) >= wellCtrl::SAVE_LITERS_DELTA;
  bool intervalReached = millis() - wellStateLastSaveMs >= wellCtrl::SAVE_INTERVAL_MS;
  bool shouldSaveTotalLiters = force || litersDeltaReached || intervalReached;

  if (!changed && !shouldSaveTotalLiters) return;

  wellPrefs.putFloat("total_liters", st.totalLiters);
  wellPrefs.putFloat("pause_ms", st.pauseMs);
  wellPrefs.putInt("failed_starts", st.failedStartCount);
  wellPrefs.putUChar("intention", static_cast<uint8_t>(st.intention));

  wellStateLastSaveMs = millis();
  wellStateLastSaveLiters = st.totalLiters;
}

void loadWellState() {
  st.totalLiters = wellPrefs.getFloat("total_liters", 0.0f);

  float storedPause = wellPrefs.getFloat("pause_ms", cfg.common.pauseMinMs);
  if (storedPause < cfg.common.pauseMinMs || storedPause > cfg.common.pauseMaxMs) {
    storedPause = cfg.common.pauseMinMs;
  }
  st.pauseMs = storedPause;

  int storedFailedStarts = wellPrefs.getInt("failed_starts", 0);
  st.failedStartCount = constrain(storedFailedStarts, 0, wellCtrl::MAX_FAILED_STARTS);

  uint8_t storedIntention = wellPrefs.getUChar("intention", static_cast<uint8_t>(PumpIntention::UNKNOWN));
  if (storedIntention > static_cast<uint8_t>(PumpIntention::PUMPING_TO_L4)) {
    storedIntention = static_cast<uint8_t>(PumpIntention::UNKNOWN);
  }
  st.intention = static_cast<PumpIntention>(storedIntention);

  persistedPauseMs = st.pauseMs;
  persistedFailedStartCount = st.failedStartCount;
  persistedIntention = st.intention;
  wellStateLastSaveLiters = st.totalLiters;
  wellStateLastSaveMs = millis();
}

void resetWellRuntimeTimers() {
  st.wellRunStart = 0;
  st.wellStartAttempt = 0;
  st.wellDryStart = 0;
  st.wellOverloadStart = 0;
  st.startCurrentOk = false;
  st.startPressureOk = false;
}

void resetWellTimersFull() {
  resetWellRuntimeTimers();
  st.wellPauseStart = 0;
}

void updateWellPumpNeed() {
  bool L2 = tm.levels[1];
  bool L4 = tm.levels[3];

  if (!L2) {
    st.needPump = true;
    st.targetOk = false;
    st.intention = PumpIntention::PUMPING_TO_L4;
  } else if (L2 && L4) {
    st.needPump = false;
    st.targetOk = true;
    st.intention = PumpIntention::TARGET_REACHED;
  } else if (L2 && !L4) {
    if (st.intention == PumpIntention::PUMPING_TO_L4) {
      st.needPump = true;
      st.targetOk = false;
    } else {
      st.needPump = false;
      st.targetOk = true;
    }
  }
}

void adjustPID(float workedMin) {
  float error = cfg.well.targetMinutes - workedMin;
  float scale = fabs(error) > wellCtrl::PID_BOOST_ERROR ? wellCtrl::PID_BOOST_FACTOR : 1.0f;
  st.pidInt += error * scale;
  st.pidInt = constrain(st.pidInt, wellCtrl::PID_INT_MIN, wellCtrl::PID_INT_MAX);

  float p = wellCtrl::PID_KP * error * scale;
  float i = wellCtrl::PID_KI * st.pidInt * scale;
  float d = wellCtrl::PID_KD * (error - st.pidLastE);
  st.pidLastE = error;

  float curMin = st.pauseMs / 60000.0f;
  curMin = constrain(curMin + p + i + d, cfg.common.pauseMinMs / 60000.0f, cfg.common.pauseMaxMs / 60000.0f);
  st.pauseMs = curMin * 60000.0f;
}

void stopWellPump(unsigned long now, const String& reason, PumpIntention nextIntention, bool withPid) {
  if (!st.wellRelay) return;
  st.wellRelay = false;

  st.lastWorkSec = st.wellRunStart ? (now - st.wellRunStart) / 1000UL : 0;
  float workedMin = st.lastWorkSec / 60.0f;
  float liters = workedMin * cfg.well.litersPerMin;
  st.totalLiters += liters;
  pushHistory(st.volumeHistory, liters);
  pushHistory(st.workHistory, st.lastWorkSec);

  if (withPid) adjustPID(workedMin);

  st.intention = nextIntention;
  st.wellPauseStart = now;
  st.wellMode = st.wellBlocked || st.pressureBlock ? WellMode::FAIL : WellMode::WAIT;
  resetWellRuntimeTimers();
  appendLog(st.logsWell, reason);
  saveWellState();
}

void initConfigFromNamespaces() {
  cfg.well = defaults::well;
  cfg.house = defaults::house;
  cfg.common = defaults::common;
  cfg.network = defaults::network;
}

WebServer server(80);

void appendLog(String& dst, const String& msg) {
  dst += msg + "\n";
  if (dst.length() > 5000) dst.remove(0, dst.length() - 5000);
}

void pushHistory(float* arr, float value) {
  for (int i = 0; i < 19; i++) arr[i] = arr[i + 1];
  arr[19] = value;
}

void parseNanoLine(const String& line) {
  if (!line.startsWith("TEL,")) return;

  float vals[11] = {0};
  int idx = 0;
  int start = 4;
  while (idx < 11 && start < (int)line.length()) {
    int comma = line.indexOf(',', start);
    if (comma < 0) comma = line.length();
    vals[idx++] = line.substring(start, comma).toFloat();
    start = comma + 1;
  }
  if (idx < 11) return;

  tm.ts = (unsigned long)vals[0];
  tm.wellCurrent = vals[1];
  tm.wellPressure = vals[2];
  tm.houseCurrent = vals[3];
  tm.housePressure = vals[4];
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
    if (c == '\n') {
      parseNanoLine(line);
      line = "";
    } else if (c != '\r') {
      line += c;
    }
  }
}

void sendNanoCommand() {
  NanoSerial.printf("RELAY=%d;VFD_RUN=%d;VFD_FREQ=%.1f\n", st.wellRelay ? 1 : 0, st.vfdRun ? 1 : 0, st.vfdFreq);
}

void runWellLogic(unsigned long now) {
  if (!tm.valid || st.wellBlocked || st.pressureBlock) {
    st.wellRelay = false;
    if (st.wellBlocked || st.pressureBlock) st.wellMode = WellMode::FAIL;
    return;
  }

  updateWellPumpNeed();
  bool needPump = st.wellForceMode || st.needPump;

  if (st.wellMode == WellMode::WAIT && needPump && !st.wellRelay && (now - st.wellPauseStart >= (unsigned long)st.pauseMs)) {
    st.wellRelay = true;
    st.wellStartAttempt = now;
    st.startCurrentOk = false;
    st.startPressureOk = false;
    st.wellMode = WellMode::STARTING;
    appendLog(st.logsWell, st.wellForceMode ? "WELL: force start" : "WELL: starting");
  }

  if (st.wellMode == WellMode::STARTING && now - st.wellStartAttempt >= wellCtrl::CURRENT_CHECK_DELAY) {
    if (tm.wellCurrent < wellCtrl::CURRENT_MIN_START) {
      st.wellRelay = false;
      st.failedStartCount++;
      saveWellState();
      st.wellPauseStart = now;
      st.wellMode = st.failedStartCount >= wellCtrl::MAX_FAILED_STARTS ? WellMode::FAIL : WellMode::WAIT;
      if (st.failedStartCount >= wellCtrl::MAX_FAILED_STARTS) {
        st.wellBlocked = true;
        st.wellAlarm = true;
        appendLog(st.logsWell, "WELL: blocked by failed starts (current)");
      } else {
        appendLog(st.logsWell, "WELL: start failed by current");
      }
      resetWellRuntimeTimers();
      return;
    }
    st.startCurrentOk = true;

    if (now - st.wellStartAttempt >= wellCtrl::PRESSURE_CHECK_DELAY) {
      if (tm.wellPressure < wellCtrl::PRESSURE_MIN_OK) {
        st.wellRelay = false;
        st.failedStartCount++;
        saveWellState();
        st.wellPauseStart = now;
        st.wellMode = st.failedStartCount >= wellCtrl::MAX_FAILED_STARTS ? WellMode::FAIL : WellMode::WAIT;
        if (st.failedStartCount >= wellCtrl::MAX_FAILED_STARTS) {
          st.wellBlocked = true;
          st.wellAlarm = true;
          appendLog(st.logsWell, "WELL: blocked by failed starts (pressure)");
        } else {
          appendLog(st.logsWell, "WELL: start failed by pressure");
        }
        resetWellRuntimeTimers();
        return;
      }
      st.startPressureOk = true;
    }

    if (st.startCurrentOk && st.startPressureOk) {
      st.wellRunStart = now;
      st.wellMode = WellMode::RUN;
      st.failedStartCount = 0;
      saveWellState();
      appendLog(st.logsWell, "WELL: run");
    }
  }

  if (st.wellMode == WellMode::RUN && !st.wellForceMode && tm.levels[1] && tm.levels[3]) {
    stopWellPump(now, "WELL: stop by L4", PumpIntention::TARGET_REACHED, true);
  }
}

void runHouseLogic() {
  if (!tm.valid || st.houseBlocked) {
    st.vfdRun = false;
    st.vfdFreq = cfg.house.minFreq;
    return;
  }

  bool hasWater = tm.levels[0] && tm.levels[1];
  if (!st.houseForceMode && !hasWater) {
    st.vfdRun = false;
    st.vfdFreq = cfg.house.minFreq;
    return;
  }

  if (st.houseForceMode) {
    st.vfdRun = true;
  } else {
    if (!st.vfdRun && tm.housePressure <= cfg.house.hystOn) {
      st.vfdRun = true;
      appendLog(st.logsHouse, "HOUSE: start");
    }
    if (st.vfdRun && tm.housePressure >= cfg.house.hystOff) {
      st.vfdRun = false;
      st.vfdFreq = cfg.house.minFreq;
      appendLog(st.logsHouse, "HOUSE: stop by pressure");
    }
  }

  if (st.vfdRun) {
    float error = cfg.house.setpointBar - tm.housePressure;
    st.vfdFreq = constrain(cfg.house.minFreq + error * 20.0f, cfg.house.minFreq, cfg.house.maxFreq);
  }
}

void runProtections(unsigned long now) {
  st.filterWarning = tm.wellPressure >= wellCtrl::PRESSURE_WARNING;
  if (tm.wellPressure >= wellCtrl::PRESSURE_BLOCK) {
    st.pressureBlock = true;
    st.wellAlarm = true;
    stopWellPump(now, "WELL: pressure block", st.intention, false);
  }

  if (st.wellRelay) {
    if (tm.wellCurrent >= cfg.well.emergencyCurrent) {
      st.wellBlocked = st.wellAlarm = true;
      st.wellRelay = false;
      st.wellForceMode = false;
      appendLog(st.logsWell, "WELL: emergency overcurrent");
    }

    if (tm.wellCurrent >= cfg.well.overloadCurrent) {
      if (!st.wellOverloadStart) st.wellOverloadStart = now;
      if (now - st.wellOverloadStart > cfg.well.overloadDelayMs) {
        st.wellBlocked = st.wellAlarm = true;
        st.wellRelay = false;
        st.wellForceMode = false;
        appendLog(st.logsWell, "WELL: overload");
      }
    } else st.wellOverloadStart = 0;

    if (tm.wellCurrent < cfg.well.dryCurrent) {
      if (!st.wellDryStart) st.wellDryStart = now;
      if (now - st.wellDryStart > cfg.well.dryDelayMs) {
        st.wellBlocked = st.wellAlarm = true;
        st.wellForceMode = false;
        stopWellPump(now, "WELL: dry run", PumpIntention::PUMPING_TO_L4, true);
      }
    } else st.wellDryStart = 0;
  }

  if (st.vfdRun) {
    if (tm.houseCurrent >= cfg.house.emergencyCurrent) {
      st.houseBlocked = st.houseAlarm = true;
      st.vfdRun = false;
      st.houseForceMode = false;
      appendLog(st.logsHouse, "HOUSE: emergency overcurrent");
    }

    if (tm.houseCurrent >= cfg.house.overloadCurrent) {
      if (!st.houseOverloadStart) st.houseOverloadStart = now;
      if (now - st.houseOverloadStart > cfg.house.overloadDelayMs) {
        st.houseBlocked = st.houseAlarm = true;
        st.vfdRun = false;
        st.houseForceMode = false;
        appendLog(st.logsHouse, "HOUSE: overload");
      }
    } else st.houseOverloadStart = 0;

    if (tm.houseCurrent < cfg.house.dryCurrent) {
      if (!st.houseDryStart) st.houseDryStart = now;
      if (now - st.houseDryStart > cfg.house.dryDelayMs) {
        st.houseBlocked = st.houseAlarm = true;
        st.vfdRun = false;
        st.houseForceMode = false;
        appendLog(st.logsHouse, "HOUSE: dry run");
      }
    } else st.houseDryStart = 0;
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
  doc["well_mode"] = (int)st.wellMode;
  doc["well_intention"] = (int)st.intention;
  doc["well_need_pump"] = st.needPump;
  doc["well_target_ok"] = st.targetOk;
  doc["well_failed_starts"] = st.failedStartCount;
  doc["filter_warning"] = st.filterWarning;
  doc["pressure_block"] = st.pressureBlock;
  doc["house_alarm"] = st.houseAlarm;
  doc["well_force"] = st.wellForceMode;
  doc["house_force"] = st.houseForceMode;
  doc["well_blocked"] = st.wellBlocked;
  doc["house_blocked"] = st.houseBlocked;
  doc["wifi_sta_connected"] = WiFi.status() == WL_CONNECTED;
  doc["wifi_sta_ip"] = WiFi.localIP().toString();
  doc["wifi_ap_ip"] = WiFi.softAPIP().toString();

  JsonArray lv = doc.createNestedArray("levels");
  for (int i = 0; i < 4; i++) lv.add(tm.levels[i]);

  JsonArray vh = doc.createNestedArray("volume_history");
  JsonArray wh = doc.createNestedArray("work_time_history");
  for (int i = 0; i < 20; i++) {
    vh.add(st.volumeHistory[i]);
    wh.add(st.workHistory[i]);
  }

  String out;
  serializeJson(doc, out);
  return out;
}

void notifyClients() {
  // kept for timing compatibility with old loop flow
}

void initWeb() {
  if (!LittleFS.begin(true)) return;

  server.on("/", HTTP_GET, []() {
    File file = LittleFS.open("/index.html", "r");
    if (!file) {
      server.send(500, "text/plain", "index.html not found in LittleFS");
      return;
    }
    server.streamFile(file, "text/html; charset=utf-8");
    file.close();
  });

  server.on("/state", HTTP_GET, []() {
    server.send(200, "application/json", buildJsonState());
  });

  server.on("/logs_well", HTTP_GET, []() {
    server.send(200, "text/plain; charset=utf-8", st.logsWell);
  });

  server.on("/logs_house", HTTP_GET, []() {
    server.send(200, "text/plain; charset=utf-8", st.logsHouse);
  });

  server.on("/set", HTTP_POST, []() {
    if (server.hasArg("param") && server.hasArg("value")) {
      String p = server.arg("param");
      float v = server.arg("value").toFloat();
      if (p == "SETPOINT_BAR") cfg.house.setpointBar = constrain(v, 0.0f, 2.0f);
      // CURRENT_DRY kept for compatibility with UI; can be persisted later.
      server.send(200, "text/plain", "OK");
      return;
    }
    server.send(400, "text/plain", "Missing param/value");
  });

  server.on("/clear_logs_well", HTTP_POST, []() {
    st.logsWell = "";
    server.send(200, "text/plain", "OK");
  });

  server.on("/reset_well", HTTP_POST, []() {
    st.wellRelay = false;
    st.wellBlocked = false;
    st.wellAlarm = false;
    st.filterWarning = false;
    st.pressureBlock = false;
    st.wellForceMode = false;
    st.wellMode = WellMode::WAIT;
    resetWellTimersFull();
    appendLog(st.logsWell, "WELL: manual reset alarms/timers");
    saveWellState(true);
    server.send(200, "text/plain", "OK");
  });

  server.on("/clear_logs_house", HTTP_POST, []() {
    st.logsHouse = "";
    server.send(200, "text/plain", "OK");
  });

  server.on("/export_well", HTTP_GET, []() {
    String csv = "idx,volume_l,work_s\n";
    for (int i = 0; i < 20; i++) csv += String(i) + "," + String(st.volumeHistory[i], 2) + "," + String(st.workHistory[i], 0) + "\n";
    server.send(200, "text/csv", csv);
  });

  server.on("/export_house", HTTP_GET, []() {
    String csv = "house_current,house_pressure\n" + String(tm.houseCurrent, 2) + "," + String(tm.housePressure, 2) + "\n";
    server.send(200, "text/csv", csv);
  });

  server.onNotFound([]() {
    String path = server.uri();
    if (LittleFS.exists(path)) {
      File file = LittleFS.open(path, "r");
      String contentType = "text/plain";
      if (path.endsWith(".css")) contentType = "text/css";
      else if (path.endsWith(".js")) contentType = "application/javascript";
      else if (path.endsWith(".html")) contentType = "text/html; charset=utf-8";
      else if (path.endsWith(".png")) contentType = "image/png";
      server.streamFile(file, contentType);
      file.close();
      return;
    }
    server.send(404, "text/plain", "Not found");
  });

  server.begin();
}

void initWiFi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(cfg.network.apSsid.c_str(), cfg.network.apPass.c_str());

  WiFi.begin(cfg.network.wifiSsid.c_str(), cfg.network.wifiPass.c_str());
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000UL) delay(300);

  if (WiFi.status() == WL_CONNECTED) {
    appendLog(st.logsWell, "Wi-Fi STA connected: " + WiFi.localIP().toString());
    appendLog(st.logsHouse, "Wi-Fi STA connected: " + WiFi.localIP().toString());
  } else {
    appendLog(st.logsWell, "Wi-Fi STA not connected, AP mode still available");
    appendLog(st.logsHouse, "Wi-Fi STA not connected, AP mode still available");
  }
  appendLog(st.logsWell, "Wi-Fi AP: " + WiFi.softAPIP().toString());
  appendLog(st.logsHouse, "Wi-Fi AP: " + WiFi.softAPIP().toString());
}

void setup() {
  Serial.begin(115200);
  NanoSerial.begin(NANO_BAUD, SERIAL_8N1, NANO_RX_PIN, NANO_TX_PIN);

  initConfigFromNamespaces();

  wellPrefs.begin(wellCtrl::PREF_NAMESPACE, false);
  loadWellState();

  initWiFi();

  initWeb();

  appendLog(st.logsWell, "System start: ESP32 controller online");
  appendLog(st.logsHouse, "System start: ESP32 controller online");
}

void loop() {
  unsigned long now = millis();

  readNanoUart();
  runWellLogic(now);
  runHouseLogic();
  runProtections(now);
  sendNanoCommand();

  static unsigned long lastWs = 0;
  if (now - lastWs > 1000) {
    lastWs = now;
    notifyClients();
  }

  saveWellState();

  server.handleClient();
  delay(20);
}
