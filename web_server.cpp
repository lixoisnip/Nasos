#include "web_server.h"

#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <WiFi.h>

#include "logging.h"
#include "settings.h"

namespace {
AsyncWebServer server(80);
Settings* cfg = nullptr;
WiFiConfig* wifiCfg = nullptr;
Telemetry* tm = nullptr;
Controller* st = nullptr;

void fillSettings(JsonObject s) {
  s["wellDryCurrent"] = cfg->wellDryCurrent;
  s["wellOverloadCurrent"] = cfg->wellOverloadCurrent;
  s["wellEmergencyCurrent"] = cfg->wellEmergencyCurrent;
  s["wellDryDelayMs"] = cfg->wellDryDelayMs;
  s["wellOverloadDelayMs"] = cfg->wellOverloadDelayMs;
  s["targetMinutes"] = cfg->targetMinutes;
  s["litersPerMin"] = cfg->litersPerMin;
  s["setpointBar"] = cfg->setpointBar;
  s["houseHystOn"] = cfg->houseHystOn;
  s["houseHystOff"] = cfg->houseHystOff;
  s["houseMinFreq"] = cfg->houseMinFreq;
  s["houseMaxFreq"] = cfg->houseMaxFreq;
  s["houseDryCurrent"] = cfg->houseDryCurrent;
  s["houseOverloadCurrent"] = cfg->houseOverloadCurrent;
  s["houseEmergencyCurrent"] = cfg->houseEmergencyCurrent;
  s["houseDryDelayMs"] = cfg->houseDryDelayMs;
  s["houseOverloadDelayMs"] = cfg->houseOverloadDelayMs;
}

void applySettingsFromJson(JsonObject in) {
  cfg->wellDryCurrent = in["wellDryCurrent"] | cfg->wellDryCurrent;
  cfg->wellOverloadCurrent = in["wellOverloadCurrent"] | cfg->wellOverloadCurrent;
  cfg->wellEmergencyCurrent = in["wellEmergencyCurrent"] | cfg->wellEmergencyCurrent;
  cfg->wellDryDelayMs = in["wellDryDelayMs"] | cfg->wellDryDelayMs;
  cfg->wellOverloadDelayMs = in["wellOverloadDelayMs"] | cfg->wellOverloadDelayMs;
  cfg->targetMinutes = in["targetMinutes"] | cfg->targetMinutes;
  cfg->litersPerMin = in["litersPerMin"] | cfg->litersPerMin;
  cfg->setpointBar = in["setpointBar"] | cfg->setpointBar;
  cfg->houseHystOn = in["houseHystOn"] | cfg->houseHystOn;
  cfg->houseHystOff = in["houseHystOff"] | cfg->houseHystOff;
  cfg->houseMinFreq = in["houseMinFreq"] | cfg->houseMinFreq;
  cfg->houseMaxFreq = in["houseMaxFreq"] | cfg->houseMaxFreq;
  cfg->houseDryCurrent = in["houseDryCurrent"] | cfg->houseDryCurrent;
  cfg->houseOverloadCurrent = in["houseOverloadCurrent"] | cfg->houseOverloadCurrent;
  cfg->houseEmergencyCurrent = in["houseEmergencyCurrent"] | cfg->houseEmergencyCurrent;
  cfg->houseDryDelayMs = in["houseDryDelayMs"] | cfg->houseDryDelayMs;
  cfg->houseOverloadDelayMs = in["houseOverloadDelayMs"] | cfg->houseOverloadDelayMs;
}
}  // namespace

String buildJsonState() {
  StaticJsonDocument<3072> doc;
  doc["well_current"] = tm->wellCurrent;
  doc["well_pressure"] = tm->wellPressure;
  doc["house_current"] = tm->houseCurrent;
  doc["house_pressure"] = tm->housePressure;
  doc["vfd_run_feedback"] = tm->vfdRunFeedback;
  doc["vfd_freq_feedback"] = tm->vfdFreqFeedback;
  doc["nano_online"] = tm->nanoOnline;
  doc["vfd_online"] = tm->vfdOnline;
  doc["wifi_connected"] = WiFi.isConnected();
  doc["last_work_sec"] = st->lastWorkSec;
  doc["pause_ms"] = st->pauseMs;
  doc["total_liters"] = st->totalLiters;
  doc["well_alarm"] = st->wellAlarm;
  doc["house_alarm"] = st->houseAlarm;
  doc["well_force"] = st->wellForceMode;
  doc["house_force"] = st->houseForceMode;
  doc["emergency_stop"] = st->emergencyStop;

  JsonArray lv = doc.createNestedArray("levels");
  for (int i = 0; i < 4; i++) lv.add(tm->levels[i]);

  JsonArray vh = doc.createNestedArray("volume_history");
  JsonArray wh = doc.createNestedArray("work_time_history");
  for (int i = 0; i < 20; i++) {
    vh.add(st->volumeHistory[i]);
    wh.add(st->workHistory[i]);
  }

  String out;
  serializeJson(doc, out);
  return out;
}

void webServerInit(Settings* cfgPtr, WiFiConfig* wifiPtr, Telemetry* tmPtr, Controller* stPtr) {
  cfg = cfgPtr;
  wifiCfg = wifiPtr;
  tm = tmPtr;
  st = stPtr;

  server.on("/state", HTTP_GET, [](AsyncWebServerRequest* req) { req->send(200, "application/json", buildJsonState()); });
  server.on("/logs_well", HTTP_GET, [](AsyncWebServerRequest* req) { req->send(200, "text/plain; charset=utf-8", readLogs(LogKind::Well)); });
  server.on("/logs_house", HTTP_GET, [](AsyncWebServerRequest* req) { req->send(200, "text/plain; charset=utf-8", readLogs(LogKind::House)); });
  server.on("/download_logs_well", HTTP_GET, [](AsyncWebServerRequest* req) { req->send(LittleFS, logPath(LogKind::Well), "text/plain", true); });
  server.on("/download_logs_house", HTTP_GET, [](AsyncWebServerRequest* req) { req->send(LittleFS, logPath(LogKind::House), "text/plain", true); });

  server.on("/clear_logs_well", HTTP_POST, [](AsyncWebServerRequest* req) {
    clearLogs(LogKind::Well);
    loggingInit();
    req->send(200, "text/plain", "OK");
  });
  server.on("/clear_logs_house", HTTP_POST, [](AsyncWebServerRequest* req) {
    clearLogs(LogKind::House);
    loggingInit();
    req->send(200, "text/plain", "OK");
  });

  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest* req) {
    StaticJsonDocument<2048> doc;
    JsonObject s = doc.createNestedObject("settings");
    fillSettings(s);
    JsonObject w = doc.createNestedObject("wifi");
    w["staSsid"] = wifiCfg->staSsid;
    w["staPass"] = wifiCfg->staPass;
    w["apSsid"] = wifiCfg->apSsid;
    w["apPass"] = wifiCfg->apPass;
    String out;
    serializeJson(doc, out);
    req->send(200, "application/json", out);
  });

  server.on("/settings", HTTP_POST, [](AsyncWebServerRequest* req) {}, nullptr,
            [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
              StaticJsonDocument<2048> doc;
              if (deserializeJson(doc, data, len)) {
                req->send(400, "text/plain", "Bad JSON");
                return;
              }
              applySettingsFromJson(doc["settings"].as<JsonObject>());
              wifiCfg->staSsid = String((const char*)doc["wifi"]["staSsid"] | wifiCfg->staSsid);
              wifiCfg->staPass = String((const char*)doc["wifi"]["staPass"] | wifiCfg->staPass);
              wifiCfg->apSsid = String((const char*)doc["wifi"]["apSsid"] | wifiCfg->apSsid);
              wifiCfg->apPass = String((const char*)doc["wifi"]["apPass"] | wifiCfg->apPass);
              validateSettings(*cfg, *wifiCfg);
              saveSettings(*cfg, *wifiCfg);
              req->send(200, "text/plain", "OK");
            });

  server.on("/pump", HTTP_POST, [](AsyncWebServerRequest* req) {
    if (!req->hasParam("unit", true) || !req->hasParam("action", true)) {
      req->send(400, "text/plain", "Missing params");
      return;
    }
    const String unit = req->getParam("unit", true)->value();
    const String action = req->getParam("action", true)->value();

    if (unit == "well") {
      if (action == "force_on") st->wellForceMode = true;
      if (action == "force_off") st->wellForceMode = false;
      if (action == "reset_alarm") st->wellBlocked = st->wellAlarm = false;
    } else if (unit == "house") {
      if (action == "force_on") st->houseForceMode = true;
      if (action == "force_off") st->houseForceMode = false;
      if (action == "reset_alarm") st->houseBlocked = st->houseAlarm = false;
    }
    req->send(200, "text/plain", "OK");
  });

  server.on("/emergency", HTTP_POST, [](AsyncWebServerRequest* req) {
    if (!req->hasParam("value", true)) {
      req->send(400, "text/plain", "Missing value");
      return;
    }
    st->emergencyStop = req->getParam("value", true)->value().toInt() == 1;
    req->send(200, "text/plain", "OK");
  });

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.begin();
}
