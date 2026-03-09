# CODEX Cleanup Report

## 1) Legacy text protocol remnants removed

- Removed unused ESP32 text-protocol parser and timestamp parser helpers from `esp32_controller.ino`:
  - `parseNanoLine(...)`
  - `ParseNanoError`
  - `isTimestampMonotonicWrapAware(...)`
- Removed obsolete text-parser diagnostics counters from `LinkHealth` and `/state` JSON:
  - `parseRejectCount`
  - `parseNoStarCount`
  - `parseBadChecksumTextCount`
  - `parseWrongFieldCount`
  - `parseInvalidNumericRangeCount`
  - `parseNonMonotonicTsCount`
- Removed Nano-side legacy command compatibility path in `v1.ino`:
  - `VERSION_LEGACY`
  - `VERSION_WITH_SETTINGS`
  - `COMMAND_FRAME_LEN_V1`, `COMMAND_FRAME_LEN_V2`
  - `NanoCommandPayloadV1`
  - mixed legacy decode branch in `applyCommandFrame(...)`

Result: one real protocol remains for Nano↔ESP32 (binary fixed-size framed protocol with CRC16 and sequence byte).

## 2) Heartbeat cleanup

- Removed separate empty heartbeat transmission from ESP32 (`sendNanoHeartbeat()` and `HEARTBEAT_PERIOD_MS`).
- Kept periodic command frames (`CMD_PERIOD_MS`) as the sole supervision traffic.

Rationale: command frames are already periodic and carry control state, so an extra heartbeat was redundant and added I2C bus noise.

## 3) Link-state semantics clarified

In `esp32_controller.ino` loop:

- `HEALTHY`: fresh telemetry and no TX error burst.
- `DEGRADED`: telemetry is still fresh but TX error burst is active (`linkHasTxErrors=true`).
- `LOST`: telemetry stale/missing, or TX path effectively unusable (outside degraded condition).

Behavior is now consistent:
- both `DEGRADED` and `LOST` force fail-safe stop (safety first),
- state transition logs now explicitly describe *why* state changed,
- duplicate action branches were consolidated.

## 4) Timestamp / freshness logic after binary migration

- Removed fake monotonic timestamp validation that was tied to old text assumptions.
- Telemetry freshness is now based on local receive time (`lastValidPacketMs` + `millis()` age checks).
- `tm.ts` continues to represent local receive time.

## 5) Diagnostics audit and alignment

Kept meaningful binary-link diagnostics:
- `txErrorCount`
- `shortReadCount`
- `invalidFrameCount`
- `badHeaderCount`
- `badCrcCount`
- `seqGapCount`
- `lastTxErrCode`
- `lastGoodRxMs`
- `lastTelemetryAgeMs`

Added explicit status flags in `/state`:
- `link_tx_error_burst`
- `link_has_tx_errors`

Removed obsolete text-only diagnostics listed in section 1.

## 6) Nano mode/state display mapping verification

- Reviewed Nano mode text mapping in `v1.ino` (`wellModeText`, `houseModeText`, `intentionText`).
- Removed legacy mode-ID branch tied to old protocol versioning.
- Updated unknown handling in `wellModeText` to return `"UNKNOWN"` instead of silently falling back to `"WAIT"`.

Result: no hidden legacy fallback for split protocol mode IDs.

## 7) Config defaults review vs `Osnova.ino`

Reviewed defaults and baseline mapping comments in `esp32_controller.ino`:
- Well/house dry-run thresholds and delays,
- pressure rise validation timing,
- startup delay,
- well pause min/max,
- current thresholds.

No additional default value change was applied in this pass; baseline defaults remain aligned with the documented `Osnova.ino` table in code comments. Existing intentionally documented house auto-restart behavior remains unchanged.

## 8) Documentation/wiring hardening

`README.md` was rewritten for clarity and technician safety:
- clearly marks `Osnova.ino` as monolithic reference only,
- explicitly states split firmware roles (`v1.ino` for Nano, `esp32_controller.ino` for ESP32),
- makes `A4/A5` I2C reservation highly visible,
- states required rewiring of legacy level inputs from `A4/A5` to `D4/D5` where applicable,
- emphasizes 3.3V/5V I2C level-shifting requirement,
- emphasizes mandatory common GND,
- documents expected fail-safe behavior on communication loss.

## 9) Final consistency cleanup

- Removed obsolete comments/identifiers that referenced legacy transport/protocol assumptions.
- Fixed outdated ESP32 header comment that incorrectly mentioned UART2 instead of I2C binary frames.
- Kept architecture and working logic intact; no major rewrite introduced.
