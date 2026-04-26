#include "control.h"

#include "logging.h"
#include "modbus.h"
#include "pins_config.h"

namespace {
Settings* cfg = nullptr;
Telemetry* tm = nullptr;
Controller* st = nullptr;
unsigned long lastModbusCmd = 0;
constexpr unsigned long MODBUS_CMD_GAP_MS = 40;
}

void controlInit(Settings* cfgPtr, Telemetry* tmPtr, Controller* stPtr) {
  cfg = cfgPtr;
  tm = tmPtr;
  st = stPtr;
}

void pushHistory(float* arr, float value) {
  for (int i = 0; i < 19; i++) arr[i] = arr[i + 1];
  arr[19] = value;
}

void runWellLogic(unsigned long now) {
  if (!tm->valid || st->wellBlocked) {
    st->wellRelay = false;
    return;
  }

  bool needByLevels = !tm->levels[3] && (!tm->levels[1] || !tm->levels[2]);
  bool needPump = st->wellForceMode || needByLevels;

  if (needPump && !st->wellRelay && (now - st->wellPauseStart > (unsigned long)st->pauseMs)) {
    st->wellRelay = true;
    st->wellRunStart = now;
    appendLog(LogKind::Well, "start", st->wellForceMode ? "WELL force start" : "WELL auto start");
  }

  if (st->wellRelay && !st->wellForceMode && tm->levels[3]) {
    st->wellRelay = false;
    st->lastWorkSec = (now - st->wellRunStart) / 1000UL;
    float liters = st->lastWorkSec * (cfg->litersPerMin / 60.0f);
    st->totalLiters += liters;
    pushHistory(st->volumeHistory, liters);
    pushHistory(st->workHistory, st->lastWorkSec);
    st->pauseMs = constrain(((cfg->targetMinutes * 60.0f) - st->lastWorkSec) * 1000.0f, 10000.0f, 90000.0f);
    st->wellPauseStart = now;
    appendLog(LogKind::Well, "stop", "WELL stop by L4");
  }
}

void runHouseLogic(unsigned long now) {
  bool prevRun = st->vfdRun;

  if (st->emergencyStop || !tm->valid || st->houseBlocked) {
    st->vfdRun = false;
    st->vfdFreq = cfg->houseMinFreq;
    return;
  }

  bool hasWater = tm->levels[0] && tm->levels[1];
  if (!st->houseForceMode && !hasWater) {
    st->vfdRun = false;
    st->vfdFreq = cfg->houseMinFreq;
    return;
  }

  if (st->houseForceMode) {
    st->vfdRun = true;
  } else {
    if (!st->vfdRun && tm->housePressure <= cfg->houseHystOn) {
      st->vfdRun = true;
      appendLog(LogKind::House, "start", "HOUSE auto start");
    }
    if (st->vfdRun && tm->housePressure >= cfg->houseHystOff) {
      st->vfdRun = false;
      st->vfdFreq = cfg->houseMinFreq;
      appendLog(LogKind::House, "stop", "HOUSE stop by pressure");
    }
  }

  if (st->vfdRun && !prevRun) st->houseRunStart = now;

  if (st->vfdRun) {
    float error = cfg->setpointBar - tm->housePressure;
    st->vfdFreq = constrain(cfg->houseMinFreq + error * 20.0f, cfg->houseMinFreq, cfg->houseMaxFreq);
  }
}

void runProtections(unsigned long now) {
  if (st->emergencyStop) {
    st->wellRelay = false;
    st->vfdRun = false;
    return;
  }

  if (st->wellRelay) {
    if (tm->wellCurrent >= cfg->wellEmergencyCurrent) {
      st->wellBlocked = st->wellAlarm = true;
      st->wellRelay = false;
      st->wellForceMode = false;
      appendLog(LogKind::Well, "alarm", "WELL emergency overcurrent");
    }
    if (tm->wellCurrent >= cfg->wellOverloadCurrent) {
      if (!st->wellOverloadStart) st->wellOverloadStart = now;
      if (now - st->wellOverloadStart > cfg->wellOverloadDelayMs) {
        st->wellBlocked = st->wellAlarm = true;
        st->wellRelay = false;
        st->wellForceMode = false;
        appendLog(LogKind::Well, "alarm", "WELL overload");
      }
    } else st->wellOverloadStart = 0;

    if (tm->wellCurrent < cfg->wellDryCurrent) {
      if (!st->wellDryStart) st->wellDryStart = now;
      if (now - st->wellDryStart > cfg->wellDryDelayMs) {
        st->wellBlocked = st->wellAlarm = true;
        st->wellRelay = false;
        st->wellForceMode = false;
        appendLog(LogKind::Well, "alarm", "WELL dry run");
      }
    } else st->wellDryStart = 0;
  }

  if (st->vfdRun) {
    if (tm->houseCurrent >= cfg->houseEmergencyCurrent) {
      st->houseBlocked = st->houseAlarm = true;
      st->vfdRun = false;
      st->houseForceMode = false;
      appendLog(LogKind::House, "alarm", "HOUSE emergency overcurrent");
    }
    if (tm->houseCurrent >= cfg->houseOverloadCurrent) {
      if (!st->houseOverloadStart) st->houseOverloadStart = now;
      if (now - st->houseOverloadStart > cfg->houseOverloadDelayMs) {
        st->houseBlocked = st->houseAlarm = true;
        st->vfdRun = false;
        st->houseForceMode = false;
        appendLog(LogKind::House, "alarm", "HOUSE overload");
      }
    } else st->houseOverloadStart = 0;

    if (tm->houseCurrent < cfg->houseDryCurrent) {
      if (!st->houseDryStart) st->houseDryStart = now;
      if (now - st->houseDryStart > cfg->houseDryDelayMs) {
        st->houseBlocked = st->houseAlarm = true;
        st->vfdRun = false;
        st->houseForceMode = false;
        appendLog(LogKind::House, "alarm", "HOUSE dry run");
      }
    } else st->houseDryStart = 0;

    if (now - st->houseRunStart > STARTUP_NO_CURRENT_DELAY_MS && tm->houseCurrent <= 0.05f) {
      st->houseBlocked = st->houseAlarm = true;
      st->vfdRun = false;
      st->houseForceMode = false;
      appendLog(LogKind::House, "alarm", "HOUSE no current after start");
    }

    if (tm->housePressure >= HOUSE_MAX_PRESSURE_TRIP_BAR) {
      st->houseBlocked = st->houseAlarm = true;
      st->vfdRun = false;
      st->houseForceMode = false;
      appendLog(LogKind::House, "alarm", "HOUSE pressure protection trip");
    }
  }
}

void applyVfdControl(unsigned long now) {
  static bool prevRun = false;
  static float prevFreq = -999.0f;

  if (now - lastModbusCmd < MODBUS_CMD_GAP_MS) return;

  if (st->vfdRun != prevRun) {
    if (vfdWriteReg(VFD_REG_RUN_CMD, st->vfdRun ? 1 : 0)) {
      prevRun = st->vfdRun;
      lastModbusCmd = now;
    }
    return;
  }

  if (fabsf(st->vfdFreq - prevFreq) >= 0.5f) {
    uint16_t cmd = (uint16_t)(constrain(st->vfdFreq, 0.0f, cfg->houseMaxFreq) * 100.0f);
    if (vfdWriteReg(VFD_REG_FREQ_CMD, cmd)) {
      prevFreq = st->vfdFreq;
      lastModbusCmd = now;
    }
  }
}
