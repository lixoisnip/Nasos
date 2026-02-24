// =====================================================
// ESP32 main controller: all pump logic/protections + web UI
// Works with Arduino Nano I/O bridge over UART2.
// =====================================================

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

// -------- Wi-Fi (edit for your network) --------
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// -------- UART to Nano --------
HardwareSerial NanoSerial(2);
constexpr int NANO_RX_PIN = 16; // ESP32 RX <- Nano TX (pin 5)
constexpr int NANO_TX_PIN = 17; // ESP32 TX -> Nano RX (pin 4)
constexpr uint32_t NANO_BAUD = 38400;

namespace cfg {
  constexpr float CURRENT_DRY = 3.3f;
  constexpr float CURRENT_OVERLOAD = 4.3f;
  constexpr float CURRENT_EMERGENCY = 6.0f;
  constexpr unsigned long DRY_DELAY_MS = 8000UL;
  constexpr unsigned long OVERLOAD_DELAY_MS = 5000UL;
  constexpr float TARGET_MINUTES = 5.0f;
  constexpr float L_PER_MIN = 30.0f;
}

namespace cfgHouse {
  constexpr float SETPOINT_BAR_DEFAULT = 1.0f;
  constexpr float HYST_ON = 0.50f;
  constexpr float HYST_OFF = 1.18f;
  constexpr float MIN_FREQ = 28.0f;
  constexpr float MAX_FREQ = 50.0f;
  constexpr float CURRENT_DRY = 0.4f;
  constexpr float CURRENT_OVERLOAD = 1.3f;
  constexpr float CURRENT_EMERGENCY = 1.5f;
  constexpr unsigned long DRY_DELAY_MS = 8000UL;
  constexpr unsigned long OVERLOAD_DELAY_MS = 5000UL;
}

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

struct Controller {
  bool wellRelay = false;
  bool vfdRun = false;
  float vfdFreq = cfgHouse::MIN_FREQ;

  bool wellBlocked = false;
  bool wellAlarm = false;
  bool houseBlocked = false;
  bool houseAlarm = false;

  unsigned long wellRunStart = 0;
  unsigned long wellPauseStart = 0;
  unsigned long wellDryStart = 0;
  unsigned long wellOverloadStart = 0;
  unsigned long houseDryStart = 0;
  unsigned long houseOverloadStart = 0;

  unsigned long lastWorkSec = 0;
  float pauseMs = 0;
  float totalLiters = 0;
  float setpointBar = cfgHouse::SETPOINT_BAR_DEFAULT;

  float volumeHistory[20] = {0};
  float workHistory[20] = {0};

  String logsWell;
  String logsHouse;
} st;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void appendLog(String& dst, const String& msg) {
  dst += msg + "\n";
  if (dst.length() > 4000) dst.remove(0, dst.length() - 4000);
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

  bool needPump = !tm.levels[3] && (!tm.levels[1] || !tm.levels[2]);

  if (needPump && !st.wellRelay && (now - st.wellPauseStart > (unsigned long)st.pauseMs)) {
    st.wellRelay = true;
    st.wellRunStart = now;
    appendLog(st.logsWell, "WELL: start");
  }

  if (st.wellRelay) {
    if (tm.levels[3]) {
      st.wellRelay = false;
      st.lastWorkSec = (now - st.wellRunStart) / 1000UL;
      float liters = st.lastWorkSec * (cfg::L_PER_MIN / 60.0f);
      st.totalLiters += liters;
      pushHistory(st.volumeHistory, liters);
      pushHistory(st.workHistory, st.lastWorkSec);
      st.pauseMs = constrain(((cfg::TARGET_MINUTES * 60.0f) - st.lastWorkSec) * 1000.0f, 10000.0f, 90000.0f);
      st.wellPauseStart = now;
      appendLog(st.logsWell, "WELL: stop by L4");
    }
  }
}

void runHouseLogic() {
  if (!tm.valid || st.houseBlocked) {
    st.vfdRun = false;
    st.vfdFreq = cfgHouse::MIN_FREQ;
    return;
  }

  bool hasWater = tm.levels[0] && tm.levels[1];
  if (!hasWater) {
    st.vfdRun = false;
    st.vfdFreq = cfgHouse::MIN_FREQ;
    return;
  }

  if (!st.vfdRun && tm.housePressure <= cfgHouse::HYST_ON) {
    st.vfdRun = true;
    appendLog(st.logsHouse, "HOUSE: start");
  }
  if (st.vfdRun && tm.housePressure >= cfgHouse::HYST_OFF) {
    st.vfdRun = false;
    st.vfdFreq = cfgHouse::MIN_FREQ;
    appendLog(st.logsHouse, "HOUSE: stop by pressure");
  }

  if (st.vfdRun) {
    float error = st.setpointBar - tm.housePressure;
    st.vfdFreq = constrain(cfgHouse::MIN_FREQ + error * 20.0f, cfgHouse::MIN_FREQ, cfgHouse::MAX_FREQ);
  }
}

void runProtections(unsigned long now) {
  if (st.wellRelay) {
    if (tm.wellCurrent >= cfg::CURRENT_EMERGENCY) {
      st.wellBlocked = st.wellAlarm = true;
      st.wellRelay = false;
      appendLog(st.logsWell, "WELL: emergency overcurrent");
    }

    if (tm.wellCurrent >= cfg::CURRENT_OVERLOAD) {
      if (!st.wellOverloadStart) st.wellOverloadStart = now;
      if (now - st.wellOverloadStart > cfg::OVERLOAD_DELAY_MS) {
        st.wellBlocked = st.wellAlarm = true;
        st.wellRelay = false;
        appendLog(st.logsWell, "WELL: overload");
      }
    } else st.wellOverloadStart = 0;

    if (tm.wellCurrent < cfg::CURRENT_DRY) {
      if (!st.wellDryStart) st.wellDryStart = now;
      if (now - st.wellDryStart > cfg::DRY_DELAY_MS) {
        st.wellBlocked = st.wellAlarm = true;
        st.wellRelay = false;
        appendLog(st.logsWell, "WELL: dry run");
      }
    } else st.wellDryStart = 0;
  }

  if (st.vfdRun) {
    if (tm.houseCurrent >= cfgHouse::CURRENT_EMERGENCY) {
      st.houseBlocked = st.houseAlarm = true;
      st.vfdRun = false;
      appendLog(st.logsHouse, "HOUSE: emergency overcurrent");
    }

    if (tm.houseCurrent >= cfgHouse::CURRENT_OVERLOAD) {
      if (!st.houseOverloadStart) st.houseOverloadStart = now;
      if (now - st.houseOverloadStart > cfgHouse::OVERLOAD_DELAY_MS) {
        st.houseBlocked = st.houseAlarm = true;
        st.vfdRun = false;
        appendLog(st.logsHouse, "HOUSE: overload");
      }
    } else st.houseOverloadStart = 0;

    if (tm.houseCurrent < cfgHouse::CURRENT_DRY) {
      if (!st.houseDryStart) st.houseDryStart = now;
      if (now - st.houseDryStart > cfgHouse::DRY_DELAY_MS) {
        st.houseBlocked = st.houseAlarm = true;
        st.vfdRun = false;
        appendLog(st.logsHouse, "HOUSE: dry run");
      }
    } else st.houseDryStart = 0;
  }
}

String buildJsonState() {
  StaticJsonDocument<2048> doc;
  doc["well_current"] = tm.wellCurrent;
  doc["well_pressure"] = tm.wellPressure;
  doc["house_current"] = tm.houseCurrent;
  doc["house_pressure"] = tm.housePressure;
  doc["last_work_sec"] = st.lastWorkSec;
  doc["pause_ms"] = st.pauseMs;
  doc["total_liters"] = st.totalLiters;
  doc["well_alarm"] = st.wellAlarm;
  doc["house_alarm"] = st.houseAlarm;

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
  ws.textAll(buildJsonState());
}

void initWeb() {
  if (!LittleFS.begin(true)) return;

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.on("/logs_well", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "text/plain; charset=utf-8", st.logsWell);
  });

  server.on("/logs_house", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "text/plain; charset=utf-8", st.logsHouse);
  });

  server.on("/set", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (req->hasParam("param") && req->hasParam("value")) {
      String p = req->getParam("param")->value();
      float v = req->getParam("value")->value().toFloat();
      if (p == "SETPOINT_BAR") st.setpointBar = constrain(v, 0.0f, 2.0f);
      // CURRENT_DRY kept for compatibility with UI; can be persisted later.
      req->send(200, "text/plain", "OK");
      return;
    }
    req->send(400, "text/plain", "Missing param/value");
  });

  server.on("/export_well", HTTP_GET, [](AsyncWebServerRequest *req) {
    String csv = "idx,volume_l,work_s\n";
    for (int i = 0; i < 20; i++) csv += String(i) + "," + String(st.volumeHistory[i], 2) + "," + String(st.workHistory[i], 0) + "\n";
    req->send(200, "text/csv", csv);
  });

  server.on("/export_house", HTTP_GET, [](AsyncWebServerRequest *req) {
    String csv = "house_current,house_pressure\n" + String(tm.houseCurrent, 2) + "," + String(tm.housePressure, 2) + "\n";
    req->send(200, "text/csv", csv);
  });

  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
                void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
      client->text(buildJsonState());
      return;
    }
    if (type == WS_EVT_DATA) {
      AwsFrameInfo *info = (AwsFrameInfo*)arg;
      if (!info->final || info->opcode != WS_TEXT) return;
      String msg;
      for (size_t i = 0; i < len; i++) msg += (char)data[i];
      if (msg == "clear_logs_well") st.logsWell = "";
      if (msg == "clear_logs_house") st.logsHouse = "";
    }
  });

  server.addHandler(&ws);
  server.begin();
}

void setup() {
  Serial.begin(115200);

  NanoSerial.begin(NANO_BAUD, SERIAL_8N1, NANO_RX_PIN, NANO_TX_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(300);

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

  ws.cleanupClients();
  delay(20);
}
