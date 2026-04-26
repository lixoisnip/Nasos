#include <LittleFS.h>
#include <WiFi.h>
#include <Wire.h>

#include "app_types.h"
#include "control.h"
#include "logging.h"
#include "modbus.h"
#include "pins_config.h"
#include "settings.h"
#include "telemetry.h"
#include "web_server.h"

Telemetry tm;
Settings cfg;
WiFiConfig wifiCfg;
Controller st;

void initWiFi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(wifiCfg.apSsid.c_str(), wifiCfg.apPass.c_str());

  if (wifiCfg.staSsid.length() > 0 && wifiCfg.staSsid != "YOUR_WIFI_SSID") {
    WiFi.begin(wifiCfg.staSsid.c_str(), wifiCfg.staPass.c_str());
  }
}

void setup() {
  Serial.begin(115200);

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  }

  loadSettings(cfg, wifiCfg);
  loggingInit();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK);
  modbusInit();

  telemetryInit(&tm, &st);
  controlInit(&cfg, &tm, &st);

  initWiFi();
  webServerInit(&cfg, &wifiCfg, &tm, &st);

  appendLog(LogKind::Well, "start", "System start: ESP32 controller online");
  appendLog(LogKind::House, "start", "System start: ESP32 controller online");
}

void loop() {
  unsigned long now = millis();

  pollNano(now);
  pollVfd(now);
  runWellLogic(now);
  runHouseLogic(now);
  runProtections(now);

  sendNanoCommand();
  applyVfdControl(now);
}
