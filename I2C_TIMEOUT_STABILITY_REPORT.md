# I2C Timeout Stability Fix Report

## Root causes of intermittent timeouts

1. **Nano I2C callbacks were doing too much work**:
   - `onReceive()` performed frame validation, header checks, CRC checks, and sequence checks.
   - This increases callback execution time and can stall slave responsiveness.
2. **ESP32 bus access cadence was aggressive**:
   - Command TX period was short and telemetry reads were attempted every main loop pass.
   - This created near back-to-back operations and elevated timeout risk.
3. **No active I2C bus recovery on repeated timeout bursts**:
   - Persistent timeout (`code=5`) scenarios did not trigger `Wire` reinit.
4. **Link event telemetry age underflow risk**:
   - Unsigned subtraction of timestamps could produce huge values if clock/reference anomalies occur.

## What was moved out of Nano callbacks

- `Wire.onReceive()` is now ISR-minimal and only:
  - copies received bytes into a shared command buffer,
  - records length,
  - sets a "ready" flag.
- `Wire.onRequest()` now only serves the latest prebuilt telemetry frame.
- Command parsing/validation moved to Nano main loop (`processEspI2c()`):
  - frame length checks,
  - header/protocol checks,
  - CRC validation,
  - sequence gap accounting,
  - command payload application.

## Telemetry prebuild behavior

- Telemetry frame continues to be built periodically in loop (`updateTelemetryFrame()` every ~100 ms).
- `onRequest()` serves only the latest ready `i2cTelemetryFrame` buffer without computation.

## ESP32 polling and pacing changes

- Reduced command polling aggressiveness:
  - `CMD_PERIOD_MS` changed from **80 ms** to **140 ms**.
- Added bounded telemetry polling cadence:
  - new `TELEMETRY_POLL_MS = 140 ms` gate in `readNanoI2c()`.
- Added small spacing guard to avoid transaction bursts:
  - new `TX_RX_GAP_MS = 12 ms` guard for command sends.

## Timeout recovery improvements on ESP32

- Added repeated-timeout detection (`txTimeoutStreak`).
- Added recovery threshold/cooldown:
  - threshold: `RECOVERY_TIMEOUT_THRESHOLD = 4`,
  - cooldown: `RECOVERY_COOLDOWN_MS = 1200 ms`.
- On threshold hit:
  - log recovery action,
  - call `Wire.end()`, short pause, then `Wire.begin(...)` with configured pins/frequency.
- Existing fail-safe behavior (stop pumps on link degradation/loss) remains intact.

## Event overflow fix (`4294967xxx` style values)

- Link event age value now uses guarded subtraction:
  - if `now >= lastValid`, use `now - lastValid`, else clamp to `0`.
- Event log payload now stores a meaningful non-underflow telemetry age value on degraded/lost transitions.

## Protocol compatibility

- Binary protocol framing, payload layout, and CRC format were **not changed**.
- Changes are behavioral/scheduling/recovery focused.
