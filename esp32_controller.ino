// =====================================================
// ESP32 main controller: all pump logic/protections + web UI
// Nano is now only an I/O expander over I2C.
// VFD RS-485 / Modbus RTU is handled directly by ESP32.
// =====================================================

#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Wire.h>

// -------- Wi-Fi settings --------
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* AP_SSID = "Nasos-ESP32";
const char* AP_PASS = "12345678";

// -------- Nano I2C --------
constexpr uint8_t NANO_I2C_ADDR = 0x10;
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint32_t I2C_CLOCK = 100000;
constexpr unsigned long NANO_POLL_MS = 120;

// -------- RS-485 / Modbus (VFD) --------
constexpr int RS485_TX_PIN = 17;   // UART2 TX -> MAX485 DI
constexpr int RS485_RX_PIN = 16;   // UART2 RX <- MAX485 RO (with divider)
constexpr int RS485_DE_PIN = 27;   // RE+DE tied together
constexpr uint32_t VFD_BAUD = 9600;
constexpr uint8_t VFD_SLAVE_ID = 0x08;
constexpr uint16_t VFD_REG_RUN_CMD = 0x9CA7;
constexpr uint16_t VFD_REG_FREQ_CMD = 0x9CA6;
constexpr uint16_t VFD_REG_OUT_FREQ_FB = 0x9CAA;
constexpr uint16_t VFD_REG_OUT_CURRENT_FB = 0x9CAB;
constexpr float VFD_FREQ_SCALE = 0.01f;
constexpr float VFD_CURRENT_SCALE = 0.01f;
constexpr unsigned long MODBUS_INTERFRAME_MS = 4;
constexpr unsigned long VFD_POLL_MS = 150;
constexpr unsigned long STARTUP_NO_CURRENT_DELAY_MS = 2500;
constexpr float HOUSE_MAX_PRESSURE_TRIP_BAR = 2.2f;

struct Telemetry {
  unsigned long ts = 0;
  float wellCurrent = 0;
  float wellPressure = 0;
  float houseCurrent = 0;
  float housePressure = 0;
  bool levels[4] = {false, false, false, false};
  bool wellRelayFeedback = false;
  bool vfdRunFeedback = false;
  float vfdFreqFeedback = 0;
  bool valid = false;
} tm;

struct Settings {
  float wellDryCurrent = 3.3f;
  float wellOverloadCurrent = 4.3f;
  float wellEmergencyCurrent = 6.0f;
  unsigned long wellDryDelayMs = 8000UL;
  unsigned long wellOverloadDelayMs = 5000UL;
  float targetMinutes = 5.0f;
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

  bool wellForceMode = false;
  bool houseForceMode = false;
  bool emergencyStop = false;

  unsigned long wellRunStart = 0;
  unsigned long wellPauseStart = 0;
  unsigned long wellDryStart = 0;
  unsigned long wellOverloadStart = 0;
  unsigned long houseDryStart = 0;
  unsigned long houseOverloadStart = 0;
  unsigned long houseRunStart = 0;

  unsigned long lastWorkSec = 0;
  float pauseMs = 0;
  float totalLiters = 0;

  float volumeHistory[20] = {0};
  float workHistory[20] = {0};

  String logsWell;
  String logsHouse;
} st;

WebServer server(80);
HardwareSerial VfdSerial(2);

#pragma pack(push, 1)
struct NanoFrame {
  uint8_t magic;
  uint32_t ms;
  int16_t wellCurrent_cA;
  int16_t wellPressure_cBar;
  int16_t housePressure_cBar;
  uint8_t levelsMask;
  uint8_t relayState;
  uint8_t crc;
};
#pragma pack(pop)

uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) crc ^= data[i];
  return crc;
}

void appendLog(String& dst, const String& msg) {
  dst += msg + "\n";
  if (dst.length() > 5000) dst.remove(0, dst.length() - 5000);
}

void pushHistory(float* arr, float value) {
  for (int i = 0; i < 19; i++) arr[i] = arr[i + 1];
  arr[19] = value;
}

uint16_t modbusCRC(const uint8_t* data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

void rs485Transmit() {
  digitalWrite(RS485_DE_PIN, HIGH);
  delayMicroseconds(120);
}

void rs485Receive() {
  digitalWrite(RS485_DE_PIN, LOW);
  delayMicroseconds(120);
}

bool vfdWriteReg(uint16_t reg, uint16_t value) {
  uint8_t req[8] = {VFD_SLAVE_ID, 0x06, (uint8_t)(reg >> 8), (uint8_t)reg, (uint8_t)(value >> 8), (uint8_t)value, 0, 0};
  uint16_t crc = modbusCRC(req, 6);
  req[6] = (uint8_t)crc;
  req[7] = (uint8_t)(crc >> 8);

  while (VfdSerial.available()) VfdSerial.read();
  rs485Transmit();
  VfdSerial.write(req, sizeof(req));
  VfdSerial.flush();
  rs485Receive();

  uint8_t resp[8] = {0};
  size_t got = VfdSerial.readBytes(resp, sizeof(resp));
  if (got != sizeof(resp)) return false;
  uint16_t rcrc = (uint16_t)resp[7] << 8 | resp[6];
  return rcrc == modbusCRC(resp, 6);
}

bool vfdReadReg(uint16_t reg, uint16_t& out) {
  uint8_t req[8] = {VFD_SLAVE_ID, 0x03, (uint8_t)(reg >> 8), (uint8_t)reg, 0x00, 0x01, 0, 0};
  uint16_t crc = modbusCRC(req, 6);
  req[6] = (uint8_t)crc;
  req[7] = (uint8_t)(crc >> 8);

  while (VfdSerial.available()) VfdSerial.read();
  rs485Transmit();
  VfdSerial.write(req, sizeof(req));
  VfdSerial.flush();
  rs485Receive();

  uint8_t resp[7] = {0};
  size_t got = VfdSerial.readBytes(resp, sizeof(resp));
  if (got != sizeof(resp) || resp[0] != VFD_SLAVE_ID || resp[1] != 0x03 || resp[2] != 0x02) return false;
  uint16_t rcrc = (uint16_t)resp[6] << 8 | resp[5];
  if (rcrc != modbusCRC(resp, 5)) return false;
  out = (uint16_t)resp[3] << 8 | resp[4];
  return true;
}

void pollNano(unsigned long now) {
  static unsigned long last = 0;
  if (now - last < NANO_POLL_MS) return;
  last = now;

  Wire.requestFrom((int)NANO_I2C_ADDR, (int)sizeof(NanoFrame));
  if (Wire.available() != (int)sizeof(NanoFrame)) return;

  NanoFrame f{};
  uint8_t* p = reinterpret_cast<uint8_t*>(&f);
  for (size_t i = 0; i < sizeof(NanoFrame); ++i) p[i] = Wire.read();
  if (f.magic != 0xA5 || crc8(p, sizeof(NanoFrame) - 1) != f.crc) return;

  tm.ts = f.ms;
  tm.wellCurrent = f.wellCurrent_cA / 100.0f;
  tm.wellPressure = f.wellPressure_cBar / 100.0f;
  tm.housePressure = f.housePressure_cBar / 100.0f;
  tm.levels[0] = f.levelsMask & 0x01;
  tm.levels[1] = f.levelsMask & 0x02;
  tm.levels[2] = f.levelsMask & 0x04;
  tm.levels[3] = f.levelsMask & 0x08;
  tm.wellRelayFeedback = f.relayState;
  tm.valid = true;
}

void sendNanoCommand() {
  Wire.beginTransmission(NANO_I2C_ADDR);
  Wire.write('C');
  Wire.write(st.wellRelay ? 1 : 0);
  Wire.endTransmission();
}

void pollVfd(unsigned long now) {
  static unsigned long last = 0;
  if (now - last < VFD_POLL_MS) return;
  last = now;

  uint16_t raw = 0;
  if (vfdReadReg(VFD_REG_OUT_CURRENT_FB, raw)) tm.houseCurrent = raw * VFD_CURRENT_SCALE;
  delay(MODBUS_INTERFRAME_MS);
  if (vfdReadReg(VFD_REG_OUT_FREQ_FB, raw)) tm.vfdFreqFeedback = raw * VFD_FREQ_SCALE;
  tm.vfdRunFeedback = tm.vfdFreqFeedback > 0.5f;
}

void applyVfdControl() {
  static bool prevRun = false;
  static float prevFreq = -999.0f;

  if (st.vfdRun != prevRun) {
    vfdWriteReg(VFD_REG_RUN_CMD, st.vfdRun ? 1 : 0);
    delay(MODBUS_INTERFRAME_MS);
    prevRun = st.vfdRun;
  }

  if (fabsf(st.vfdFreq - prevFreq) >= 0.1f) {
    uint16_t cmd = (uint16_t)(constrain(st.vfdFreq, 0.0f, cfg.houseMaxFreq) * 100.0f);
    vfdWriteReg(VFD_REG_FREQ_CMD, cmd);
    delay(MODBUS_INTERFRAME_MS);
    prevFreq = st.vfdFreq;
  }
}

void runWellLogic(unsigned long now) {
  if (!tm.valid || st.wellBlocked) {
    st.wellRelay = false;
    return;
  }

  bool needByLevels = !tm.levels[3] && (!tm.levels[1] || !tm.levels[2]);
  bool needPump = st.wellForceMode || needByLevels;

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

void runHouseLogic(unsigned long now) {
  bool prevRun = st.vfdRun;

  if (st.emergencyStop || !tm.valid || st.houseBlocked) {
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

  if (st.vfdRun && !prevRun) st.houseRunStart = now;

  if (st.vfdRun) {
    float error = cfg.setpointBar - tm.housePressure;
    st.vfdFreq = constrain(cfg.houseMinFreq + error * 20.0f, cfg.houseMinFreq, cfg.houseMaxFreq);
  }
}

void runProtections(unsigned long now) {
  if (st.emergencyStop) {
    st.wellRelay = false;
    st.vfdRun = false;
    return;
  }

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

    if (now - st.houseRunStart > STARTUP_NO_CURRENT_DELAY_MS && tm.houseCurrent <= 0.05f) {
      st.houseBlocked = st.houseAlarm = true;
      st.vfdRun = false;
      st.houseForceMode = false;
      appendLog(st.logsHouse, "HOUSE: no current after start");
    }

    if (tm.housePressure >= HOUSE_MAX_PRESSURE_TRIP_BAR) {
      st.houseBlocked = st.houseAlarm = true;
      st.vfdRun = false;
      st.houseForceMode = false;
      appendLog(st.logsHouse, "HOUSE: pressure protection trip");
    }
  }
}

String buildJsonState() {
  StaticJsonDocument<3072> doc;
  doc["well_current"] = tm.wellCurrent;
  doc["well_pressure"] = tm.wellPressure;
  doc["house_current"] = tm.houseCurrent;
  doc["house_pressure"] = tm.housePressure;
  doc["vfd_run_feedback"] = tm.vfdRunFeedback;
  doc["vfd_freq_feedback"] = tm.vfdFreqFeedback;
  doc["last_work_sec"] = st.lastWorkSec;
  doc["pause_ms"] = st.pauseMs;
  doc["total_liters"] = st.totalLiters;
  doc["well_alarm"] = st.wellAlarm;
  doc["house_alarm"] = st.houseAlarm;
  doc["emergency_stop"] = st.emergencyStop;

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

  server.on("/state", HTTP_GET, []() { server.send(200, "application/json", buildJsonState()); });
  server.on("/logs_well", HTTP_GET, []() { server.send(200, "text/plain; charset=utf-8", st.logsWell); });
  server.on("/logs_house", HTTP_GET, []() { server.send(200, "text/plain; charset=utf-8", st.logsHouse); });

  server.on("/set", HTTP_POST, []() {
    if (server.hasArg("param") && server.hasArg("value")) {
      String p = server.arg("param");
      float v = server.arg("value").toFloat();
      if (p == "SETPOINT_BAR") cfg.setpointBar = constrain(v, 0.0f, 2.0f);
      server.send(200, "text/plain", "OK");
      return;
    }
    server.send(400, "text/plain", "Missing param/value");
  });

  server.on("/emergency", HTTP_POST, []() {
    if (!server.hasArg("value")) {
      server.send(400, "text/plain", "Missing value");
      return;
    }
    st.emergencyStop = server.arg("value").toInt() == 1;
    if (st.emergencyStop) {
      st.wellRelay = false;
      st.vfdRun = false;
      appendLog(st.logsWell, "WELL: emergency stop");
      appendLog(st.logsHouse, "HOUSE: emergency stop");
    }
    server.send(200, "text/plain", "OK");
  });

  server.begin();
}

void initWiFi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK);

  pinMode(RS485_DE_PIN, OUTPUT);
  rs485Receive();
  VfdSerial.begin(VFD_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  initWiFi();
  initWeb();

  appendLog(st.logsWell, "System start: ESP32 controller online");
  appendLog(st.logsHouse, "System start: ESP32 controller online");
}

void loop() {
  unsigned long now = millis();

  pollNano(now);
  pollVfd(now);
  runWellLogic(now);
  runHouseLogic(now);
  runProtections(now);

  sendNanoCommand();
  applyVfdControl();

  server.handleClient();
  delay(20);
}
