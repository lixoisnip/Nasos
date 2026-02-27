// =====================================================
// ESP32 main controller: all pump logic/protections + web UI
// Works with Arduino Nano I/O bridge over UART2.
// =====================================================

#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

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
  unsigned long levelFilterMs;
  unsigned long initDelayMs;
  float thresh;
};

struct NetworkConfig {
  String wifiSsid;
  String wifiPass;
  String apSsid;
  String apPass;
};


namespace watchdogCfg {
constexpr uint32_t TIMEOUT_S = 10;
}

void feedTaskWatchdog() {
  esp_task_wdt_reset();
}

void initTaskWatchdog() {
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
  const esp_task_wdt_config_t twdtConfig = {
    .timeout_ms = watchdogCfg::TIMEOUT_S * 1000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true
  };
  esp_task_wdt_init(&twdtConfig);
#else
  esp_task_wdt_init(watchdogCfg::TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);
}

String resetReasonToString(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "Power-on reset";
    case ESP_RST_EXT: return "External reset";
    case ESP_RST_SW: return "Software reset";
    case ESP_RST_PANIC: return "Exception/panic reset";
    case ESP_RST_INT_WDT: return "Interrupt watchdog reset";
    case ESP_RST_TASK_WDT: return "Task watchdog reset";
    case ESP_RST_WDT: return "Other watchdog reset";
    case ESP_RST_DEEPSLEEP: return "Wakeup from deep sleep";
    case ESP_RST_BROWNOUT: return "Brownout reset";
    case ESP_RST_SDIO: return "SDIO reset";
    default: return "Unknown reset reason";
  }
}

void logResetReason() {
  const esp_reset_reason_t reason = esp_reset_reason();
  const String message = "Причина перезапуска: " + resetReasonToString(reason) + " (" + String((int)reason) + ")";
  appendLog(st.logsWell, message);
  appendLog(st.logsHouse, message);
  Serial.println(message);
}

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
  90000.0f,
  200UL,
  3000UL,
  0.10f
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

namespace houseCtrl {
constexpr unsigned long PID_PERIOD = 300UL;
constexpr float PID_KP = 18.0f;
constexpr float PID_KI = 2.8f;
constexpr float INTEGRAL_LIMIT = 6.0f;
constexpr float FREQ_STEP = 1.5f;
constexpr unsigned long FREQ_STEP_DELAY = 180UL;

constexpr float PRESSURE_RISE_THRESHOLD = 0.08f;
constexpr unsigned long DRY_PRESSURE_START_TIMEOUT = 8000UL;
constexpr unsigned long DRY_PRESSURE_WORK_TIMEOUT = 12000UL;

constexpr unsigned long START_CURRENT_IGNORE_MS = 2500UL;
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

enum class HouseMode : uint8_t {
  WAIT_WATER,
  READY,
  RUNNING,
  STOPPED
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

struct LinkHealth {
  unsigned long lastValidPacketMs = 0;
  unsigned long crcErrorCount = 0;
  unsigned long totalPackets = 0;
} linkHealth;

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
  unsigned long houseStartAt = 0;
  unsigned long housePidLastAt = 0;
  unsigned long houseFreqLastStepAt = 0;
  unsigned long housePressureDryStartAt = 0;
  float houseInitialPressure = 0;
  bool housePressureRiseOk = false;
  float housePidIntegral = 0;
  float houseTargetFreq = 28.0f;
  HouseMode houseMode = HouseMode::WAIT_WATER;

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
Preferences settingsPrefs;
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


void saveSettingsToPrefs() {
  settingsPrefs.putFloat("w_dry_cur", cfg.well.dryCurrent);
  settingsPrefs.putFloat("w_over_cur", cfg.well.overloadCurrent);
  settingsPrefs.putFloat("w_em_cur", cfg.well.emergencyCurrent);
  settingsPrefs.putULong("w_dry_ms", cfg.well.dryDelayMs);
  settingsPrefs.putULong("w_over_ms", cfg.well.overloadDelayMs);
  settingsPrefs.putFloat("w_tgt_min", cfg.well.targetMinutes);
  settingsPrefs.putFloat("w_lpm", cfg.well.litersPerMin);

  settingsPrefs.putFloat("h_set", cfg.house.setpointBar);
  settingsPrefs.putFloat("h_on", cfg.house.hystOn);
  settingsPrefs.putFloat("h_off", cfg.house.hystOff);
  settingsPrefs.putFloat("h_minf", cfg.house.minFreq);
  settingsPrefs.putFloat("h_maxf", cfg.house.maxFreq);
  settingsPrefs.putFloat("h_dry_cur", cfg.house.dryCurrent);
  settingsPrefs.putFloat("h_over_cur", cfg.house.overloadCurrent);
  settingsPrefs.putFloat("h_em_cur", cfg.house.emergencyCurrent);
  settingsPrefs.putULong("h_dry_ms", cfg.house.dryDelayMs);
  settingsPrefs.putULong("h_over_ms", cfg.house.overloadDelayMs);

  settingsPrefs.putFloat("c_pmin", cfg.common.pauseMinMs);
  settingsPrefs.putFloat("c_pmax", cfg.common.pauseMaxMs);
  settingsPrefs.putULong("c_lvl_ms", cfg.common.levelFilterMs);
  settingsPrefs.putULong("c_init_ms", cfg.common.initDelayMs);
  settingsPrefs.putFloat("c_thresh", cfg.common.thresh);
}

void loadSettingsFromPrefs() {
  cfg.well.dryCurrent = settingsPrefs.getFloat("w_dry_cur", cfg.well.dryCurrent);
  cfg.well.overloadCurrent = settingsPrefs.getFloat("w_over_cur", cfg.well.overloadCurrent);
  cfg.well.emergencyCurrent = settingsPrefs.getFloat("w_em_cur", cfg.well.emergencyCurrent);
  cfg.well.dryDelayMs = settingsPrefs.getULong("w_dry_ms", cfg.well.dryDelayMs);
  cfg.well.overloadDelayMs = settingsPrefs.getULong("w_over_ms", cfg.well.overloadDelayMs);
  cfg.well.targetMinutes = settingsPrefs.getFloat("w_tgt_min", cfg.well.targetMinutes);
  cfg.well.litersPerMin = settingsPrefs.getFloat("w_lpm", cfg.well.litersPerMin);

  cfg.house.setpointBar = settingsPrefs.getFloat("h_set", cfg.house.setpointBar);
  cfg.house.hystOn = settingsPrefs.getFloat("h_on", cfg.house.hystOn);
  cfg.house.hystOff = settingsPrefs.getFloat("h_off", cfg.house.hystOff);
  cfg.house.minFreq = settingsPrefs.getFloat("h_minf", cfg.house.minFreq);
  cfg.house.maxFreq = settingsPrefs.getFloat("h_maxf", cfg.house.maxFreq);
  cfg.house.dryCurrent = settingsPrefs.getFloat("h_dry_cur", cfg.house.dryCurrent);
  cfg.house.overloadCurrent = settingsPrefs.getFloat("h_over_cur", cfg.house.overloadCurrent);
  cfg.house.emergencyCurrent = settingsPrefs.getFloat("h_em_cur", cfg.house.emergencyCurrent);
  cfg.house.dryDelayMs = settingsPrefs.getULong("h_dry_ms", cfg.house.dryDelayMs);
  cfg.house.overloadDelayMs = settingsPrefs.getULong("h_over_ms", cfg.house.overloadDelayMs);

  cfg.common.pauseMinMs = settingsPrefs.getFloat("c_pmin", cfg.common.pauseMinMs);
  cfg.common.pauseMaxMs = settingsPrefs.getFloat("c_pmax", cfg.common.pauseMaxMs);
  cfg.common.levelFilterMs = settingsPrefs.getULong("c_lvl_ms", cfg.common.levelFilterMs);
  cfg.common.initDelayMs = settingsPrefs.getULong("c_init_ms", cfg.common.initDelayMs);
  cfg.common.thresh = settingsPrefs.getFloat("c_thresh", cfg.common.thresh);
}

bool applySingleSetting(const String& key, float value) {
  if (key == "SETPOINT_BAR" || key == "cfgHouse.setpointBar") cfg.house.setpointBar = constrain(value, 0.0f, 4.0f);
  else if (key == "CURRENT_DRY" || key == "cfg.dryCurrent") cfg.well.dryCurrent = constrain(value, 0.0f, 20.0f);
  else if (key == "cfg.overloadCurrent") cfg.well.overloadCurrent = constrain(value, 0.0f, 30.0f);
  else if (key == "cfg.emergencyCurrent") cfg.well.emergencyCurrent = constrain(value, 0.0f, 40.0f);
  else if (key == "cfg.dryDelayMs") cfg.well.dryDelayMs = (unsigned long)constrain(value, 100.0f, 120000.0f);
  else if (key == "cfg.overloadDelayMs") cfg.well.overloadDelayMs = (unsigned long)constrain(value, 100.0f, 120000.0f);
  else if (key == "cfg.targetMinutes") cfg.well.targetMinutes = constrain(value, 0.1f, 60.0f);
  else if (key == "cfg.litersPerMin") cfg.well.litersPerMin = constrain(value, 0.1f, 300.0f);

  else if (key == "cfgHouse.hystOn") cfg.house.hystOn = constrain(value, 0.1f, 4.0f);
  else if (key == "cfgHouse.hystOff") cfg.house.hystOff = constrain(value, 0.1f, 4.0f);
  else if (key == "cfgHouse.minFreq") cfg.house.minFreq = constrain(value, 10.0f, 60.0f);
  else if (key == "cfgHouse.maxFreq") cfg.house.maxFreq = constrain(value, 10.0f, 60.0f);
  else if (key == "cfgHouse.dryCurrent") cfg.house.dryCurrent = constrain(value, 0.0f, 20.0f);
  else if (key == "cfgHouse.overloadCurrent") cfg.house.overloadCurrent = constrain(value, 0.0f, 30.0f);
  else if (key == "cfgHouse.emergencyCurrent") cfg.house.emergencyCurrent = constrain(value, 0.0f, 40.0f);
  else if (key == "cfgHouse.dryDelayMs") cfg.house.dryDelayMs = (unsigned long)constrain(value, 100.0f, 120000.0f);
  else if (key == "cfgHouse.overloadDelayMs") cfg.house.overloadDelayMs = (unsigned long)constrain(value, 100.0f, 120000.0f);

  else if (key == "common.pauseMinMs") cfg.common.pauseMinMs = constrain(value, 1000.0f, 300000.0f);
  else if (key == "common.pauseMaxMs") cfg.common.pauseMaxMs = constrain(value, 1000.0f, 600000.0f);
  else if (key == "LEVEL_FILTER_MS" || key == "common.LEVEL_FILTER_MS") cfg.common.levelFilterMs = (unsigned long)constrain(value, 0.0f, 30000.0f);
  else if (key == "INIT_DELAY_MS" || key == "common.INIT_DELAY_MS") cfg.common.initDelayMs = (unsigned long)constrain(value, 0.0f, 60000.0f);
  else if (key == "THRESH" || key == "common.THRESH") cfg.common.thresh = constrain(value, 0.0f, 5.0f);
  else return false;

  if (cfg.house.minFreq > cfg.house.maxFreq) cfg.house.maxFreq = cfg.house.minFreq;
  if (cfg.common.pauseMinMs > cfg.common.pauseMaxMs) cfg.common.pauseMaxMs = cfg.common.pauseMinMs;
  return true;
}

String buildJsonSettings() {
  StaticJsonDocument<2048> doc;
  JsonObject c1 = doc.createNestedObject("cfg");
  c1["dryCurrent"] = cfg.well.dryCurrent;
  c1["overloadCurrent"] = cfg.well.overloadCurrent;
  c1["emergencyCurrent"] = cfg.well.emergencyCurrent;
  c1["dryDelayMs"] = cfg.well.dryDelayMs;
  c1["overloadDelayMs"] = cfg.well.overloadDelayMs;
  c1["targetMinutes"] = cfg.well.targetMinutes;
  c1["litersPerMin"] = cfg.well.litersPerMin;

  JsonObject c2 = doc.createNestedObject("cfgHouse");
  c2["setpointBar"] = cfg.house.setpointBar;
  c2["hystOn"] = cfg.house.hystOn;
  c2["hystOff"] = cfg.house.hystOff;
  c2["minFreq"] = cfg.house.minFreq;
  c2["maxFreq"] = cfg.house.maxFreq;
  c2["dryCurrent"] = cfg.house.dryCurrent;
  c2["overloadCurrent"] = cfg.house.overloadCurrent;
  c2["emergencyCurrent"] = cfg.house.emergencyCurrent;
  c2["dryDelayMs"] = cfg.house.dryDelayMs;
  c2["overloadDelayMs"] = cfg.house.overloadDelayMs;

  JsonObject c3 = doc.createNestedObject("common");
  c3["pauseMinMs"] = cfg.common.pauseMinMs;
  c3["pauseMaxMs"] = cfg.common.pauseMaxMs;
  c3["LEVEL_FILTER_MS"] = cfg.common.levelFilterMs;
  c3["INIT_DELAY_MS"] = cfg.common.initDelayMs;
  c3["THRESH"] = cfg.common.thresh;

  String out;
  serializeJson(doc, out);
  return out;
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

uint8_t calcXorChecksum(const String& payload) {
  uint8_t checksum = 0;
  for (int i = 0; i < payload.length(); i++) checksum ^= (uint8_t)payload[i];
  return checksum;
}

void parseNanoLine(const String& line) {
  int starPos = line.lastIndexOf('*');
  if (starPos <= 0 || starPos + 2 >= (int)line.length()) return;

  String payload = line.substring(0, starPos);
  String checksumText = line.substring(starPos + 1);
  checksumText.trim();
  uint8_t expected = (uint8_t)strtoul(checksumText.c_str(), nullptr, 16);
  uint8_t actual = calcXorChecksum(payload);

  linkHealth.totalPackets++;
  if (actual != expected) {
    linkHealth.crcErrorCount++;
    return;
  }

  if (!payload.startsWith("TEL,")) return;

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
  tm.housePressure = vals[4];
  tm.levels[0] = vals[5] > 0.5f;
  tm.levels[1] = vals[6] > 0.5f;
  tm.levels[2] = vals[7] > 0.5f;
  tm.levels[3] = vals[8] > 0.5f;
  tm.vfdRunFeedback = vals[9] > 0.5f;
  tm.vfdFreqFeedback = vals[10];
  tm.valid = true;
  linkHealth.lastValidPacketMs = millis();
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
  NanoSerial.printf(
    "RELAY=%d;VFD_RUN=%d;VFD_FREQ=%.1f;WELL_MODE=%d;WELL_ALARM=%d;WELL_BLOCKED=%d;WELL_INTENTION=%d;HOUSE_MODE=%d;HOUSE_ALARM=%d;HOUSE_BLOCKED=%d\n",
    st.wellRelay ? 1 : 0,
    st.vfdRun ? 1 : 0,
    st.vfdFreq,
    (int)st.wellMode,
    st.wellAlarm ? 1 : 0,
    st.wellBlocked ? 1 : 0,
    (int)st.intention,
    (int)st.houseMode,
    st.houseAlarm ? 1 : 0,
    st.houseBlocked ? 1 : 0
  );
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
    appendLog(st.logsWell, st.wellForceMode ? "Скважина: принудительный запуск" : "Скважина: запуск");
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
        appendLog(st.logsWell, "Скважина: блокировка — нет тока после 3 попыток запуска");
      } else {
        appendLog(st.logsWell, "Скважина: неудачный запуск — ток не появился");
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
          appendLog(st.logsWell, "Скважина: блокировка — давление не выросло после 3 попыток");
        } else {
          appendLog(st.logsWell, "Скважина: неудачный запуск — давление не выросло");
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
      appendLog(st.logsWell, "Скважина: насос работает");
    }
  }

  if (st.wellMode == WellMode::RUN && !st.wellForceMode && tm.levels[1] && tm.levels[3]) {
    stopWellPump(now, "Скважина: остановка — достигнут верхний уровень L4", PumpIntention::TARGET_REACHED, true);
  }
}

void runHouseLogic() {
  if (!tm.valid || st.houseBlocked) {
    st.vfdRun = false;
    st.vfdFreq = cfg.house.minFreq;
    st.houseMode = HouseMode::STOPPED;
    st.housePressureDryStartAt = 0;
    st.housePidLastAt = 0;
    st.houseStartAt = 0;
    st.housePressureRiseOk = false;
    return;
  }

  bool L1 = tm.levels[0];
  bool L2 = tm.levels[1];
  bool fullWater = L1 && L2;

  if (!st.houseForceMode) {
    if (!L1) {
      st.houseMode = HouseMode::WAIT_WATER;
      st.vfdRun = false;
      st.vfdFreq = cfg.house.minFreq;
      st.housePressureDryStartAt = 0;
      st.housePidLastAt = 0;
      return;
    }

    if (L1 && !L2) {
      if (!st.vfdRun) {
        st.houseMode = HouseMode::READY;
      }
    } else {
      st.houseMode = st.vfdRun ? HouseMode::RUNNING : HouseMode::READY;
    }
  } else {
    st.houseMode = st.vfdRun ? HouseMode::RUNNING : HouseMode::READY;
  }

  if (!st.houseForceMode && !fullWater && !st.vfdRun) {
    st.vfdRun = false;
    st.vfdFreq = cfg.house.minFreq;
    return;
  }

  if (st.houseForceMode) {
    st.vfdRun = true;
  } else {
    if (!st.vfdRun && tm.housePressure <= cfg.house.hystOn) {
      st.vfdRun = true;
      st.houseMode = HouseMode::RUNNING;
      st.houseStartAt = millis();
      st.houseInitialPressure = tm.housePressure;
      st.housePressureRiseOk = false;
      st.housePressureDryStartAt = 0;
      st.housePidLastAt = 0;
      st.houseFreqLastStepAt = 0;
      st.housePidIntegral = 0;
      st.houseTargetFreq = cfg.house.minFreq;
      appendLog(st.logsHouse, "Дом: запуск насоса");
    }
    if (st.vfdRun && tm.housePressure >= cfg.house.hystOff) {
      st.vfdRun = false;
      st.houseMode = HouseMode::READY;
      st.vfdFreq = cfg.house.minFreq;
      st.housePressureDryStartAt = 0;
      st.housePidLastAt = 0;
      appendLog(st.logsHouse, "Дом: остановка — верхний порог давления достигнут");
    }
  }

  if (st.vfdRun) {
    if (!st.houseStartAt) {
      st.houseStartAt = millis();
      st.houseInitialPressure = tm.housePressure;
      st.housePressureRiseOk = false;
    }

    unsigned long now = millis();
    if (!st.housePidLastAt) st.housePidLastAt = now;

    if (now - st.housePidLastAt >= houseCtrl::PID_PERIOD) {
      float dt = (now - st.housePidLastAt) / 1000.0f;
      st.housePidLastAt = now;
      float error = cfg.house.setpointBar - tm.housePressure;
      st.housePidIntegral += error * dt;
      st.housePidIntegral = constrain(st.housePidIntegral, -houseCtrl::INTEGRAL_LIMIT, houseCtrl::INTEGRAL_LIMIT);
      st.houseTargetFreq = cfg.house.minFreq + houseCtrl::PID_KP * error + houseCtrl::PID_KI * st.housePidIntegral;
      st.houseTargetFreq = constrain(st.houseTargetFreq, cfg.house.minFreq, cfg.house.maxFreq);
    }

    if (!st.houseFreqLastStepAt || now - st.houseFreqLastStepAt >= houseCtrl::FREQ_STEP_DELAY) {
      st.houseFreqLastStepAt = now;
      float delta = st.houseTargetFreq - st.vfdFreq;
      if (fabs(delta) <= houseCtrl::FREQ_STEP) st.vfdFreq = st.houseTargetFreq;
      else st.vfdFreq += delta > 0 ? houseCtrl::FREQ_STEP : -houseCtrl::FREQ_STEP;
      st.vfdFreq = constrain(st.vfdFreq, cfg.house.minFreq, cfg.house.maxFreq);
    }
  } else {
    st.housePidLastAt = 0;
    st.housePressureDryStartAt = 0;
  }
}

void runProtections(unsigned long now) {
  st.filterWarning = tm.wellPressure >= wellCtrl::PRESSURE_WARNING;
  if (tm.wellPressure >= wellCtrl::PRESSURE_BLOCK) {
    st.pressureBlock = true;
    st.wellAlarm = true;
    stopWellPump(now, "Скважина: авария — давление выше порога, работа запрещена", st.intention, false);
  }

  if (st.wellRelay) {
    if (tm.wellCurrent >= cfg.well.emergencyCurrent) {
      st.wellBlocked = st.wellAlarm = true;
      st.wellRelay = false;
      st.wellForceMode = false;
      appendLog(st.logsWell, "Скважина: авария — аварийная перегрузка по току");
    }

    if (tm.wellCurrent >= cfg.well.overloadCurrent) {
      if (!st.wellOverloadStart) st.wellOverloadStart = now;
      if (now - st.wellOverloadStart > cfg.well.overloadDelayMs) {
        st.wellBlocked = st.wellAlarm = true;
        st.wellRelay = false;
        st.wellForceMode = false;
        appendLog(st.logsWell, "Скважина: авария — перегрузка по току");
      }
    } else st.wellOverloadStart = 0;

    if (tm.wellCurrent < cfg.well.dryCurrent) {
      if (!st.wellDryStart) st.wellDryStart = now;
      if (now - st.wellDryStart > cfg.well.dryDelayMs) {
        st.wellBlocked = st.wellAlarm = true;
        st.wellForceMode = false;
        stopWellPump(now, "Скважина: сухой ход — ток ниже порога", PumpIntention::PUMPING_TO_L4, true);
      }
    } else st.wellDryStart = 0;
  }

  if (st.vfdRun) {
    bool ignoreStartCurrent = st.houseStartAt && (now - st.houseStartAt < houseCtrl::START_CURRENT_IGNORE_MS);

    if (tm.houseCurrent >= cfg.house.emergencyCurrent) {
      st.houseBlocked = st.houseAlarm = true;
      st.vfdRun = false;
      st.houseForceMode = false;
      st.houseMode = HouseMode::STOPPED;
      appendLog(st.logsHouse, "Дом: авария — аварийная перегрузка по току");
    }

    if (!ignoreStartCurrent && tm.houseCurrent >= cfg.house.overloadCurrent) {
      if (!st.houseOverloadStart) st.houseOverloadStart = now;
      if (now - st.houseOverloadStart > cfg.house.overloadDelayMs) {
        st.houseBlocked = st.houseAlarm = true;
        st.vfdRun = false;
        st.houseForceMode = false;
        st.houseMode = HouseMode::STOPPED;
        appendLog(st.logsHouse, "Дом: авария — перегрузка по току");
      }
    } else st.houseOverloadStart = 0;

    if (!ignoreStartCurrent && tm.houseCurrent < cfg.house.dryCurrent) {
      if (!st.houseDryStart) st.houseDryStart = now;
      if (now - st.houseDryStart > cfg.house.dryDelayMs) {
        st.houseBlocked = st.houseAlarm = true;
        st.vfdRun = false;
        st.houseForceMode = false;
        st.houseMode = HouseMode::STOPPED;
        appendLog(st.logsHouse, "Дом: сухой ход — ток ниже порога");
      }
    } else st.houseDryStart = 0;

    if (!st.housePressureRiseOk && st.houseStartAt) {
      if (tm.housePressure > st.houseInitialPressure + houseCtrl::PRESSURE_RISE_THRESHOLD) {
        st.housePressureRiseOk = true;
      } else if (now - st.houseStartAt >= houseCtrl::DRY_PRESSURE_START_TIMEOUT) {
        st.houseBlocked = st.houseAlarm = true;
        st.vfdRun = false;
        st.houseForceMode = false;
        st.houseMode = HouseMode::STOPPED;
        appendLog(st.logsHouse, "Дом: сухой ход — давление не выросло за 8 с после старта");
      }
    }

    if (tm.housePressure <= cfg.house.hystOn) {
      if (!st.housePressureDryStartAt) st.housePressureDryStartAt = now;
      if (now - st.housePressureDryStartAt > houseCtrl::DRY_PRESSURE_WORK_TIMEOUT) {
        st.houseBlocked = st.houseAlarm = true;
        st.vfdRun = false;
        st.houseForceMode = false;
        st.houseMode = HouseMode::STOPPED;
        appendLog(st.logsHouse, "Дом: сухой ход — давление не выросло за 10 с");
      }
    } else {
      st.housePressureDryStartAt = 0;
    }
  } else {
    st.houseOverloadStart = 0;
    st.houseDryStart = 0;
    st.housePressureDryStartAt = 0;
    st.houseStartAt = 0;
    st.housePressureRiseOk = false;
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
  doc["house_mode"] = (int)st.houseMode;
  doc["well_force"] = st.wellForceMode;
  doc["house_force"] = st.houseForceMode;
  doc["well_blocked"] = st.wellBlocked;
  doc["house_blocked"] = st.houseBlocked;
  doc["wifi_sta_connected"] = WiFi.status() == WL_CONNECTED;
  doc["wifi_sta_ip"] = WiFi.localIP().toString();
  doc["wifi_ap_ip"] = WiFi.softAPIP().toString();
  doc["link_last_valid_ms"] = linkHealth.lastValidPacketMs;
  doc["link_crc_errors"] = linkHealth.crcErrorCount;
  doc["link_total_packets"] = linkHealth.totalPackets;

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

  server.on("/settings", HTTP_GET, []() {
    server.send(200, "application/json", buildJsonSettings());
  });

  server.on("/set", HTTP_POST, []() {
    if (!server.hasArg("param") || !server.hasArg("value")) {
      server.send(400, "text/plain", "Missing param/value");
      return;
    }
    String p = server.arg("param");
    float v = server.arg("value").toFloat();
    if (!applySingleSetting(p, v)) {
      server.send(400, "text/plain", "Unknown parameter");
      return;
    }
    saveSettingsToPrefs();
    server.send(200, "application/json", buildJsonSettings());
  });

  server.on("/settings/save", HTTP_POST, []() {
    if (!server.hasArg("plain")) {
      server.send(400, "text/plain", "Missing JSON body");
      return;
    }

    StaticJsonDocument<3072> doc;
    DeserializationError err = deserializeJson(doc, server.arg("plain"));
    if (err) {
      server.send(400, "text/plain", "Bad JSON");
      return;
    }

    JsonObject cfgWell = doc["cfg"];
    for (JsonPair kv : cfgWell) applySingleSetting(String("cfg.") + kv.key().c_str(), kv.value().as<float>());

    JsonObject cfgHouse = doc["cfgHouse"];
    for (JsonPair kv : cfgHouse) applySingleSetting(String("cfgHouse.") + kv.key().c_str(), kv.value().as<float>());

    JsonObject common = doc["common"];
    for (JsonPair kv : common) applySingleSetting(String("common.") + kv.key().c_str(), kv.value().as<float>());

    saveSettingsToPrefs();
    server.send(200, "application/json", buildJsonSettings());
  });

  server.on("/pump_action", HTTP_POST, []() {
    String pump = server.arg("pump");
    String action = server.arg("action");

    if (pump == "well") {
      if (action == "reset_alarm") {
        st.wellRelay = false;
        st.wellBlocked = false;
        st.wellAlarm = false;
        st.filterWarning = false;
        st.pressureBlock = false;
        st.wellForceMode = false;
        st.wellMode = WellMode::WAIT;
        resetWellTimersFull();
        appendLog(st.logsWell, "Скважина: ручной сброс аварии");
        saveWellState(true);
      } else if (action == "force_on") {
        st.wellForceMode = true;
        appendLog(st.logsWell, "Скважина: включен принудительный режим");
      } else if (action == "force_off") {
        st.wellForceMode = false;
        if (st.wellRelay) stopWellPump(millis(), "Скважина: принудительный режим отключен", st.intention, false);
      } else {
        server.send(400, "text/plain", "Unknown action");
        return;
      }
    } else if (pump == "house") {
      if (action == "reset_alarm") {
        st.houseBlocked = false;
        st.houseAlarm = false;
        st.houseForceMode = false;
        st.houseMode = HouseMode::READY;
        appendLog(st.logsHouse, "Дом: ручной сброс аварии");
      } else if (action == "force_on") {
        st.houseForceMode = true;
        appendLog(st.logsHouse, "Дом: включен принудительный режим");
      } else if (action == "force_off") {
        st.houseForceMode = false;
        st.vfdRun = false;
        st.houseMode = HouseMode::STOPPED;
        appendLog(st.logsHouse, "Дом: принудительный режим отключен");
      } else {
        server.send(400, "text/plain", "Unknown action");
        return;
      }
    } else {
      server.send(400, "text/plain", "Unknown pump");
      return;
    }

    server.send(200, "text/plain", "OK");
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
    appendLog(st.logsWell, "Скважина: ручной сброс аварии и таймеров");
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
    appendLog(st.logsWell, "Wi-Fi STA подключен: " + WiFi.localIP().toString());
    appendLog(st.logsHouse, "Wi-Fi STA подключен: " + WiFi.localIP().toString());
  } else {
    appendLog(st.logsWell, "Wi-Fi STA не подключен, доступен режим AP");
    appendLog(st.logsHouse, "Wi-Fi STA не подключен, доступен режим AP");
  }
  appendLog(st.logsWell, "Wi-Fi AP: " + WiFi.softAPIP().toString());
  appendLog(st.logsHouse, "Wi-Fi AP: " + WiFi.softAPIP().toString());
}

void setup() {
  Serial.begin(115200);
  NanoSerial.begin(NANO_BAUD, SERIAL_8N1, NANO_RX_PIN, NANO_TX_PIN);

  initConfigFromNamespaces();

  settingsPrefs.begin("settings", false);
  loadSettingsFromPrefs();

  wellPrefs.begin(wellCtrl::PREF_NAMESPACE, false);
  loadWellState();

  initWiFi();

  initWeb();

  initTaskWatchdog();
  feedTaskWatchdog();
  logResetReason();

  appendLog(st.logsWell, "Система запущена: контроллер ESP32 онлайн");
  appendLog(st.logsHouse, "Система запущена: контроллер ESP32 онлайн");
}

void loop() {
  feedTaskWatchdog();
  unsigned long now = millis();

  readNanoUart();
  feedTaskWatchdog();
  runWellLogic(now);
  runHouseLogic();
  runProtections(now);
  sendNanoCommand();
  feedTaskWatchdog();

  static unsigned long lastWs = 0;
  if (now - lastWs > 1000) {
    lastWs = now;
    notifyClients();
  }

  saveWellState();

  server.handleClient();
  feedTaskWatchdog();
}
