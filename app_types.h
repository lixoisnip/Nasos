#pragma once

#include <Arduino.h>

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
  bool nanoOnline = false;
  bool vfdOnline = false;
};

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
};

struct WiFiConfig {
  String staSsid = "YOUR_WIFI_SSID";
  String staPass = "YOUR_WIFI_PASSWORD";
  String apSsid = "Nasos-ESP32";
  String apPass = "12345678";
};

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
};
