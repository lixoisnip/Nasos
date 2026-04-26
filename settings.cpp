#include "settings.h"

#include <ArduinoJson.h>
#include <LittleFS.h>

namespace {
constexpr const char* SETTINGS_PATH = "/settings.json";
}

void setFactoryDefaults(Settings& cfg, WiFiConfig& wifi) {
  cfg = Settings{};
  wifi = WiFiConfig{};
}

bool validateSettings(Settings& cfg, WiFiConfig& wifi) {
  bool valid = true;
  if (cfg.houseMinFreq < 0 || cfg.houseMinFreq > 60) { cfg.houseMinFreq = 28; valid = false; }
  if (cfg.houseMaxFreq < cfg.houseMinFreq || cfg.houseMaxFreq > 70) { cfg.houseMaxFreq = 50; valid = false; }
  if (cfg.houseHystOn > cfg.houseHystOff) { cfg.houseHystOn = 0.5f; cfg.houseHystOff = 1.18f; valid = false; }
  if (cfg.wellDryCurrent <= 0 || cfg.wellEmergencyCurrent < cfg.wellOverloadCurrent) { cfg.wellDryCurrent = 3.3f; valid = false; }
  if (wifi.apSsid.length() < 1) { wifi.apSsid = "Nasos-ESP32"; valid = false; }
  if (wifi.apPass.length() < 8) { wifi.apPass = "12345678"; valid = false; }
  return valid;
}

bool saveSettings(const Settings& cfg, const WiFiConfig& wifi) {
  StaticJsonDocument<2048> doc;
  JsonObject s = doc.createNestedObject("settings");
  s["wellDryCurrent"] = cfg.wellDryCurrent;
  s["wellOverloadCurrent"] = cfg.wellOverloadCurrent;
  s["wellEmergencyCurrent"] = cfg.wellEmergencyCurrent;
  s["wellDryDelayMs"] = cfg.wellDryDelayMs;
  s["wellOverloadDelayMs"] = cfg.wellOverloadDelayMs;
  s["targetMinutes"] = cfg.targetMinutes;
  s["litersPerMin"] = cfg.litersPerMin;
  s["setpointBar"] = cfg.setpointBar;
  s["houseHystOn"] = cfg.houseHystOn;
  s["houseHystOff"] = cfg.houseHystOff;
  s["houseMinFreq"] = cfg.houseMinFreq;
  s["houseMaxFreq"] = cfg.houseMaxFreq;
  s["houseDryCurrent"] = cfg.houseDryCurrent;
  s["houseOverloadCurrent"] = cfg.houseOverloadCurrent;
  s["houseEmergencyCurrent"] = cfg.houseEmergencyCurrent;
  s["houseDryDelayMs"] = cfg.houseDryDelayMs;
  s["houseOverloadDelayMs"] = cfg.houseOverloadDelayMs;

  JsonObject w = doc.createNestedObject("wifi");
  w["staSsid"] = wifi.staSsid;
  w["staPass"] = wifi.staPass;
  w["apSsid"] = wifi.apSsid;
  w["apPass"] = wifi.apPass;

  File f = LittleFS.open(SETTINGS_PATH, FILE_WRITE);
  if (!f) return false;
  serializeJsonPretty(doc, f);
  f.close();
  return true;
}

bool loadSettings(Settings& cfg, WiFiConfig& wifi) {
  if (!LittleFS.exists(SETTINGS_PATH)) {
    setFactoryDefaults(cfg, wifi);
    return saveSettings(cfg, wifi);
  }

  File f = LittleFS.open(SETTINGS_PATH, FILE_READ);
  if (!f) {
    setFactoryDefaults(cfg, wifi);
    return false;
  }

  StaticJsonDocument<2048> doc;
  auto err = deserializeJson(doc, f);
  f.close();
  if (err) {
    setFactoryDefaults(cfg, wifi);
    saveSettings(cfg, wifi);
    return false;
  }

  JsonObject s = doc["settings"];
  JsonObject w = doc["wifi"];

  cfg.wellDryCurrent = s["wellDryCurrent"] | cfg.wellDryCurrent;
  cfg.wellOverloadCurrent = s["wellOverloadCurrent"] | cfg.wellOverloadCurrent;
  cfg.wellEmergencyCurrent = s["wellEmergencyCurrent"] | cfg.wellEmergencyCurrent;
  cfg.wellDryDelayMs = s["wellDryDelayMs"] | cfg.wellDryDelayMs;
  cfg.wellOverloadDelayMs = s["wellOverloadDelayMs"] | cfg.wellOverloadDelayMs;
  cfg.targetMinutes = s["targetMinutes"] | cfg.targetMinutes;
  cfg.litersPerMin = s["litersPerMin"] | cfg.litersPerMin;
  cfg.setpointBar = s["setpointBar"] | cfg.setpointBar;
  cfg.houseHystOn = s["houseHystOn"] | cfg.houseHystOn;
  cfg.houseHystOff = s["houseHystOff"] | cfg.houseHystOff;
  cfg.houseMinFreq = s["houseMinFreq"] | cfg.houseMinFreq;
  cfg.houseMaxFreq = s["houseMaxFreq"] | cfg.houseMaxFreq;
  cfg.houseDryCurrent = s["houseDryCurrent"] | cfg.houseDryCurrent;
  cfg.houseOverloadCurrent = s["houseOverloadCurrent"] | cfg.houseOverloadCurrent;
  cfg.houseEmergencyCurrent = s["houseEmergencyCurrent"] | cfg.houseEmergencyCurrent;
  cfg.houseDryDelayMs = s["houseDryDelayMs"] | cfg.houseDryDelayMs;
  cfg.houseOverloadDelayMs = s["houseOverloadDelayMs"] | cfg.houseOverloadDelayMs;

  wifi.staSsid = String((const char*)w["staSsid"] | wifi.staSsid);
  wifi.staPass = String((const char*)w["staPass"] | wifi.staPass);
  wifi.apSsid = String((const char*)w["apSsid"] | wifi.apSsid);
  wifi.apPass = String((const char*)w["apPass"] | wifi.apPass);

  if (!validateSettings(cfg, wifi)) saveSettings(cfg, wifi);
  return true;
}
