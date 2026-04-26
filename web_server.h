#pragma once

#include <Arduino.h>

#include "app_types.h"

void webServerInit(Settings* cfg, WiFiConfig* wifi, Telemetry* tm, Controller* st);
String buildJsonState();
