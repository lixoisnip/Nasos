#pragma once

#include <Arduino.h>

enum class LogKind { Well, House };

void loggingInit();
void appendLog(LogKind kind, const String& eventType, const String& message);
String readLogs(LogKind kind);
bool clearLogs(LogKind kind);
const char* logPath(LogKind kind);
