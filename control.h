#pragma once

#include "app_types.h"

void controlInit(Settings* cfg, Telemetry* tm, Controller* st);
void runWellLogic(unsigned long now);
void runHouseLogic(unsigned long now);
void runProtections(unsigned long now);
void applyVfdControl(unsigned long now);
void pushHistory(float* arr, float value);
