#pragma once

#include "app_types.h"

void telemetryInit(Telemetry* tm, Controller* st);
void pollNano(unsigned long now);
void pollVfd(unsigned long now);
void sendNanoCommand();
