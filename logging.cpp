#include "logging.h"

#include <LittleFS.h>

namespace {
constexpr size_t MAX_LOG_SIZE = 16 * 1024;

const char* pathFor(LogKind kind) {
  return kind == LogKind::Well ? "/logs_well.txt" : "/logs_house.txt";
}

String isoTimestamp() {
  unsigned long s = millis() / 1000;
  char buf[32];
  snprintf(buf, sizeof(buf), "T+%lus", s);
  return String(buf);
}

void rotateIfNeeded(const char* path) {
  File f = LittleFS.open(path, FILE_READ);
  if (!f) return;
  size_t size = f.size();
  if (size <= MAX_LOG_SIZE) {
    f.close();
    return;
  }

  const size_t keep = MAX_LOG_SIZE / 2;
  f.seek(size - keep, SeekSet);
  String tail = f.readString();
  f.close();

  File w = LittleFS.open(path, FILE_WRITE);
  if (!w) return;
  w.print(tail);
  w.close();
}
}  // namespace

void loggingInit() {
  if (!LittleFS.exists(pathFor(LogKind::Well))) {
    File f = LittleFS.open(pathFor(LogKind::Well), FILE_WRITE);
    if (f) f.close();
  }
  if (!LittleFS.exists(pathFor(LogKind::House))) {
    File f = LittleFS.open(pathFor(LogKind::House), FILE_WRITE);
    if (f) f.close();
  }
}

void appendLog(LogKind kind, const String& eventType, const String& message) {
  const char* path = pathFor(kind);
  File f = LittleFS.open(path, FILE_APPEND);
  if (!f) return;
  f.printf("[%s] (%s) %s\n", isoTimestamp().c_str(), eventType.c_str(), message.c_str());
  f.close();
  rotateIfNeeded(path);
}

String readLogs(LogKind kind) {
  File f = LittleFS.open(pathFor(kind), FILE_READ);
  if (!f) return "";
  String out = f.readString();
  f.close();
  return out;
}

bool clearLogs(LogKind kind) {
  return LittleFS.remove(pathFor(kind));
}

const char* logPath(LogKind kind) { return pathFor(kind); }
