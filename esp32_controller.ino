#if defined(ARDUINO_ARCH_ESP32)
// =====================================================
// ESP32 main controller: all pump logic/protections + web UI
// Works with Arduino Nano I/O bridge over UART2.
// =====================================================

#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

// -------- Wi-Fi defaults (compatibility names) --------
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* AP_SSID = "Nasos-ESP32";
const char* AP_PASS = "12345678";  // min 8 chars for WPA2

// Backward-compatible aliases for older revisions / copied sketches.
const char* WIFI_SSID_DEFAULT = WIFI_SSID;
const char* WIFI_PASS_DEFAULT = WIFI_PASS;
const char* AP_SSID_DEFAULT = AP_SSID;
const char* AP_PASS_DEFAULT = AP_PASS;

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
  // Well pump
  float wellDryCurrent = 3.3f;
  float wellOverloadCurrent = 4.3f;
  float wellEmergencyCurrent = 6.0f;
  unsigned long wellDryDelayMs = 8000UL;
  unsigned long wellOverloadDelayMs = 5000UL;
  float targetMinutes = 5.0f;
  float litersPerMin = 30.0f;

  // House pump
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

  // Sensor ranges (for calibration/display settings)
  float wellPressureMinBar = 0.0f;
  float wellPressureMaxBar = 12.0f;
  float housePressureMinBar = 0.0f;
  float housePressureMaxBar = 12.0f;
  float wellCurrentMinA = 0.0f;
  float wellCurrentMaxA = 10.0f;
  float houseCurrentMinA = 0.0f;
  float houseCurrentMaxA = 10.0f;

  // Wi-Fi
  String wifiSsid = WIFI_SSID;
  String wifiPass = WIFI_PASS;
  String apSsid = AP_SSID;
  String apPass = AP_PASS;  // min 8 chars for WPA2
} cfg;

struct Controller {
  bool wellRelay = false;
  bool vfdRun = false;
  float vfdFreq = 28.0f;

  bool wellBlocked = false;
  bool wellAlarm = false;
  bool houseBlocked = false;
  bool houseAlarm = false;

  bool wellForceMode = false;
  bool houseForceMode = false;

  unsigned long wellRunStart = 0;
  unsigned long wellPauseStart = 0;
  unsigned long wellDryStart = 0;
  unsigned long wellOverloadStart = 0;
  unsigned long houseDryStart = 0;
  unsigned long houseOverloadStart = 0;

  unsigned long lastWorkSec = 0;
  float pauseMs = 0;
  float totalLiters = 0;

  // Backward compatibility for older code paths.
  float setpointBar = 1.0f;               // legacy member name used in some old lambdas
  float setpointBarCompatibility = 1.0f;  // runtime value is cfg.setpointBar

  float volumeHistory[20] = {0};
  float workHistory[20] = {0};

  String logsWell;
  String logsHouse;
} st;

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
  if (!tm.valid || st.wellBlocked) {
    st.wellRelay = false;
    return;
  }

  bool needByLevels = !tm.levels[3] && (!tm.levels[1] || !tm.levels[2]);
  bool needPump = st.wellForceMode || needByLevels;

  // Force mode bypasses level logic, but pause timer and protections still have priority.
  if (needPump && !st.wellRelay && (now - st.wellPauseStart > (unsigned long)st.pauseMs)) {
    st.wellRelay = true;
    st.wellRunStart = now;
    appendLog(st.logsWell, st.wellForceMode ? "WELL: force start" : "WELL: start");
  }

  if (st.wellRelay && !st.wellForceMode && tm.levels[3]) {
    st.wellRelay = false;
    st.lastWorkSec = (now - st.wellRunStart) / 1000UL;
    float liters = st.lastWorkSec * (cfg.litersPerMin / 60.0f);
    st.totalLiters += liters;
    pushHistory(st.volumeHistory, liters);
    pushHistory(st.workHistory, st.lastWorkSec);
    st.pauseMs = constrain(((cfg.targetMinutes * 60.0f) - st.lastWorkSec) * 1000.0f, 10000.0f, 90000.0f);
    st.wellPauseStart = now;
    appendLog(st.logsWell, "WELL: stop by L4");
  }
}

void runHouseLogic() {
  if (!tm.valid || st.houseBlocked) {
    st.vfdRun = false;
    st.vfdFreq = cfg.houseMinFreq;
    return;
  }

  bool hasWater = tm.levels[0] && tm.levels[1];
  if (!st.houseForceMode && !hasWater) {
    st.vfdRun = false;
    st.vfdFreq = cfg.houseMinFreq;
    return;
  }

  if (st.houseForceMode) {
    st.vfdRun = true;
  } else {
    if (!st.vfdRun && tm.housePressure <= cfg.houseHystOn) {
      st.vfdRun = true;
      appendLog(st.logsHouse, "HOUSE: start");
    }
    if (st.vfdRun && tm.housePressure >= cfg.houseHystOff) {
      st.vfdRun = false;
      st.vfdFreq = cfg.houseMinFreq;
      appendLog(st.logsHouse, "HOUSE: stop by pressure");
    }
  }

  if (st.vfdRun) {
    float error = cfg.setpointBar - tm.housePressure;
    st.vfdFreq = constrain(cfg.houseMinFreq + error * 20.0f, cfg.houseMinFreq, cfg.houseMaxFreq);
  }
}

void runProtections(unsigned long now) {
  if (st.wellRelay) {
    if (tm.wellCurrent >= cfg.wellEmergencyCurrent) {
      st.wellBlocked = st.wellAlarm = true;
      st.wellRelay = false;
      st.wellForceMode = false;
      appendLog(st.logsWell, "WELL: emergency overcurrent");
    }

    if (tm.wellCurrent >= cfg.wellOverloadCurrent) {
      if (!st.wellOverloadStart) st.wellOverloadStart = now;
      if (now - st.wellOverloadStart > cfg.wellOverloadDelayMs) {
        st.wellBlocked = st.wellAlarm = true;
        st.wellRelay = false;
        st.wellForceMode = false;
        appendLog(st.logsWell, "WELL: overload");
      }
    } else st.wellOverloadStart = 0;

    if (tm.wellCurrent < cfg.wellDryCurrent) {
      if (!st.wellDryStart) st.wellDryStart = now;
      if (now - st.wellDryStart > cfg.wellDryDelayMs) {
        st.wellBlocked = st.wellAlarm = true;
        st.wellRelay = false;
        st.wellForceMode = false;
        appendLog(st.logsWell, "WELL: dry run");
      }
    } else st.wellDryStart = 0;
  }

  if (st.vfdRun) {
    if (tm.houseCurrent >= cfg.houseEmergencyCurrent) {
      st.houseBlocked = st.houseAlarm = true;
      st.vfdRun = false;
      st.houseForceMode = false;
      appendLog(st.logsHouse, "HOUSE: emergency overcurrent");
    }

    if (tm.houseCurrent >= cfg.houseOverloadCurrent) {
      if (!st.houseOverloadStart) st.houseOverloadStart = now;
      if (now - st.houseOverloadStart > cfg.houseOverloadDelayMs) {
        st.houseBlocked = st.houseAlarm = true;
        st.vfdRun = false;
        st.houseForceMode = false;
        appendLog(st.logsHouse, "HOUSE: overload");
      }
    } else st.houseOverloadStart = 0;

    if (tm.houseCurrent < cfg.houseDryCurrent) {
      if (!st.houseDryStart) st.houseDryStart = now;
      if (now - st.houseDryStart > cfg.houseDryDelayMs) {
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

String buildJsonSettings() {
  StaticJsonDocument<3072> doc;
  doc["well_dry_current"] = cfg.wellDryCurrent;
  doc["well_overload_current"] = cfg.wellOverloadCurrent;
  doc["well_emergency_current"] = cfg.wellEmergencyCurrent;
  doc["well_dry_delay_ms"] = cfg.wellDryDelayMs;
  doc["well_overload_delay_ms"] = cfg.wellOverloadDelayMs;
  doc["target_minutes"] = cfg.targetMinutes;
  doc["liters_per_min"] = cfg.litersPerMin;

  doc["setpoint_bar"] = cfg.setpointBar;
  doc["house_hyst_on"] = cfg.houseHystOn;
  doc["house_hyst_off"] = cfg.houseHystOff;
  doc["house_min_freq"] = cfg.houseMinFreq;
  doc["house_max_freq"] = cfg.houseMaxFreq;
  doc["house_dry_current"] = cfg.houseDryCurrent;
  doc["house_overload_current"] = cfg.houseOverloadCurrent;
  doc["house_emergency_current"] = cfg.houseEmergencyCurrent;
  doc["house_dry_delay_ms"] = cfg.houseDryDelayMs;
  doc["house_overload_delay_ms"] = cfg.houseOverloadDelayMs;

  doc["well_pressure_min_bar"] = cfg.wellPressureMinBar;
  doc["well_pressure_max_bar"] = cfg.wellPressureMaxBar;
  doc["house_pressure_min_bar"] = cfg.housePressureMinBar;
  doc["house_pressure_max_bar"] = cfg.housePressureMaxBar;
  doc["well_current_min_a"] = cfg.wellCurrentMinA;
  doc["well_current_max_a"] = cfg.wellCurrentMaxA;
  doc["house_current_min_a"] = cfg.houseCurrentMinA;
  doc["house_current_max_a"] = cfg.houseCurrentMaxA;

  doc["wifi_ssid"] = cfg.wifiSsid;
  doc["wifi_pass"] = cfg.wifiPass;
  doc["ap_ssid"] = cfg.apSsid;
  doc["ap_pass"] = cfg.apPass;

  String out;
  serializeJson(doc, out);
  return out;
}

void setFloatParam(const String& p, float v) {
  if (p == "well_dry_current") cfg.wellDryCurrent = constrain(v, 0.0f, 30.0f);
  else if (p == "well_overload_current") cfg.wellOverloadCurrent = constrain(v, 0.0f, 30.0f);
  else if (p == "well_emergency_current") cfg.wellEmergencyCurrent = constrain(v, 0.0f, 30.0f);
  else if (p == "well_dry_delay_ms") cfg.wellDryDelayMs = (unsigned long)constrain(v, 100.0f, 120000.0f);
  else if (p == "well_overload_delay_ms") cfg.wellOverloadDelayMs = (unsigned long)constrain(v, 100.0f, 120000.0f);
  else if (p == "target_minutes") cfg.targetMinutes = constrain(v, 0.1f, 30.0f);
  else if (p == "liters_per_min") cfg.litersPerMin = constrain(v, 1.0f, 200.0f);
  else if (p == "setpoint_bar") cfg.setpointBar = constrain(v, 0.0f, 10.0f);
  else if (p == "house_hyst_on") cfg.houseHystOn = constrain(v, 0.0f, 10.0f);
  else if (p == "house_hyst_off") cfg.houseHystOff = constrain(v, 0.0f, 10.0f);
  else if (p == "house_min_freq") cfg.houseMinFreq = constrain(v, 0.0f, 100.0f);
  else if (p == "house_max_freq") cfg.houseMaxFreq = constrain(v, 0.0f, 100.0f);
  else if (p == "house_dry_current") cfg.houseDryCurrent = constrain(v, 0.0f, 30.0f);
  else if (p == "house_overload_current") cfg.houseOverloadCurrent = constrain(v, 0.0f, 30.0f);
  else if (p == "house_emergency_current") cfg.houseEmergencyCurrent = constrain(v, 0.0f, 30.0f);
  else if (p == "house_dry_delay_ms") cfg.houseDryDelayMs = (unsigned long)constrain(v, 100.0f, 120000.0f);
  else if (p == "house_overload_delay_ms") cfg.houseOverloadDelayMs = (unsigned long)constrain(v, 100.0f, 120000.0f);
  else if (p == "well_pressure_min_bar") cfg.wellPressureMinBar = constrain(v, 0.0f, 20.0f);
  else if (p == "well_pressure_max_bar") cfg.wellPressureMaxBar = constrain(v, 0.0f, 20.0f);
  else if (p == "house_pressure_min_bar") cfg.housePressureMinBar = constrain(v, 0.0f, 20.0f);
  else if (p == "house_pressure_max_bar") cfg.housePressureMaxBar = constrain(v, 0.0f, 20.0f);
  else if (p == "well_current_min_a") cfg.wellCurrentMinA = constrain(v, 0.0f, 50.0f);
  else if (p == "well_current_max_a") cfg.wellCurrentMaxA = constrain(v, 0.0f, 50.0f);
  else if (p == "house_current_min_a") cfg.houseCurrentMinA = constrain(v, 0.0f, 50.0f);
  else if (p == "house_current_max_a") cfg.houseCurrentMaxA = constrain(v, 0.0f, 50.0f);

  if (cfg.houseMaxFreq < cfg.houseMinFreq) cfg.houseMaxFreq = cfg.houseMinFreq;
  if (cfg.houseHystOff < cfg.houseHystOn) cfg.houseHystOff = cfg.houseHystOn;
}

void initWiFi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(cfg.apSsid.c_str(), cfg.apPass.c_str());

  WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPass.c_str());
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000UL) delay(300);

  if (WiFi.status() == WL_CONNECTED) {
    appendLog(st.logsWell, "Wi-Fi STA connected: " + WiFi.localIP().toString());
  } else {
    appendLog(st.logsWell, "Wi-Fi STA not connected, AP mode still available");
  }
  appendLog(st.logsWell, "Wi-Fi AP: " + WiFi.softAPIP().toString());
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

  server.on("/settings", HTTP_GET, []() {
    server.send(200, "application/json", buildJsonSettings());
  });

  server.on("/set", HTTP_POST, []() {
    if (!(server.hasArg("param") && server.hasArg("value"))) {
      server.send(400, "text/plain", "Missing param/value");
      return;
    }

    String p = server.arg("param");
    if (p == "wifi_ssid") cfg.wifiSsid = server.arg("value");
    else if (p == "wifi_pass") cfg.wifiPass = server.arg("value");
    else if (p == "ap_ssid") cfg.apSsid = server.arg("value");
    else if (p == "ap_pass") cfg.apPass = server.arg("value");
    else setFloatParam(p, server.arg("value").toFloat());

    server.send(200, "text/plain", "OK");
  });

  server.on("/action", HTTP_POST, []() {
    String pump = server.arg("pump");
    String cmd = server.arg("cmd");

    if (pump == "well" && cmd == "reset_alarm") {
      st.wellAlarm = false;
      st.wellBlocked = false;
      st.wellDryStart = st.wellOverloadStart = 0;
      appendLog(st.logsWell, "WELL: alarm reset");
      server.send(200, "text/plain", "OK");
      return;
    }
    if (pump == "house" && cmd == "reset_alarm") {
      st.houseAlarm = false;
      st.houseBlocked = false;
      st.houseDryStart = st.houseOverloadStart = 0;
      appendLog(st.logsHouse, "HOUSE: alarm reset");
      server.send(200, "text/plain", "OK");
      return;
    }
    if (pump == "well" && cmd == "force_on") {
      st.wellForceMode = true;
      appendLog(st.logsWell, "WELL: force mode ON");
      server.send(200, "text/plain", "OK");
      return;
    }
    if (pump == "well" && cmd == "force_off") {
      st.wellForceMode = false;
      appendLog(st.logsWell, "WELL: force mode OFF");
      server.send(200, "text/plain", "OK");
      return;
    }
    if (pump == "house" && cmd == "force_on") {
      st.houseForceMode = true;
      appendLog(st.logsHouse, "HOUSE: force mode ON");
      server.send(200, "text/plain", "OK");
      return;
    }
    if (pump == "house" && cmd == "force_off") {
      st.houseForceMode = false;
      appendLog(st.logsHouse, "HOUSE: force mode OFF");
      server.send(200, "text/plain", "OK");
      return;
    }

    server.send(400, "text/plain", "Unknown action");
  });

  server.on("/logs_well", HTTP_GET, []() { server.send(200, "text/plain; charset=utf-8", st.logsWell); });
  server.on("/logs_house", HTTP_GET, []() { server.send(200, "text/plain; charset=utf-8", st.logsHouse); });
  server.on("/clear_logs_well", HTTP_POST, []() { st.logsWell = ""; server.send(200, "text/plain", "OK"); });
  server.on("/clear_logs_house", HTTP_POST, []() { st.logsHouse = ""; server.send(200, "text/plain", "OK"); });

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

void setup() {
  Serial.begin(115200);
  NanoSerial.begin(NANO_BAUD, SERIAL_8N1, NANO_RX_PIN, NANO_TX_PIN);

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

  server.handleClient();
  delay(20);
}
#endif
