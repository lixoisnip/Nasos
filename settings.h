#pragma once

#include <Arduino.h>
#include "app_types.h"

bool loadSettings(Settings& cfg, WiFiConfig& wifi);
bool saveSettings(const Settings& cfg, const WiFiConfig& wifi);
void setFactoryDefaults(Settings& cfg, WiFiConfig& wifi);
bool validateSettings(Settings& cfg, WiFiConfig& wifi);
