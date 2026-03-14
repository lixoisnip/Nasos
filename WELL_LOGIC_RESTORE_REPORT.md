# WELL LOGIC RESTORE REPORT

## Scope
Compared the original monolithic reference (`Osnova.ino`) with the split ESP32 controller (`esp32_controller.ino`) and restored well pump operational semantics around RUN dry-stop, startup failure separation, pressure/filter behavior priority, and Nano link-loss handling.

## Structured comparison summary (`Osnova.ino` vs split logic)

### 1) Refill demand and L2/L4 intent handling
- **Reference (`Osnova.ino`)**: `updateWellPumpNeed()` uses L2/L4 with sticky intent: no L2 => pump needed, L2+L4 => target reached, L2 without L4 continues pumping only if prior intention was pumping-to-L4.
- **Restored split logic**: same behavior retained in ESP32 `updateWellPumpNeed()`.

### 2) Startup validation (strict and separate from RUN dry-stop)
- **Reference**: STARTING validates current and pressure after startup delays; repeated failed starts can block.
- **Restored split logic**: STARTING continues to enforce current/pressure checks and increment failed-start counter; block/alarm only after repeated failed starts.

### 3) Dry-run during successful RUN
- **Reference**: RUN dry condition is a normal operational stop, runtime/liters counted, pause adapted, intention remains `PUMPING_TO_L4`, transition to WAIT without latch alarm/block.
- **Fix applied**: split logic now performs normal `stopWellPump(..., PUMPING_TO_L4, withPid=true)` on dry in RUN **without** setting `wellBlocked`/`wellAlarm`.

### 4) Pressure warning / pressure block / filter warning
- **Reference**: warning and block are distinct; warning indicates filter/high-pressure condition while block is hard protection.
- **Restored split logic**: `filterWarning` remains warning-only at `PRESSURE_WARNING`; `pressureBlock` remains hard-stop at `PRESSURE_BLOCK` with alarm semantics.

### 5) WAIT vs FAIL transition philosophy
- **Reference**: FAIL reserved for hard faults (overcurrent emergency/overload, hard pressure block, repeated startup failures), not for normal RUN dry-stop.
- **Restored split logic**: RUN dry now routes to WAIT path; hard faults remain FAIL/BLOCK.

### 6) Nano/UART communication loss semantics
- **Reference philosophy**: communication channel faults are not hydraulic faults.
- **Fix applied**: on link degraded/lost, outputs are safely stopped and telemetry invalidated, but well hydraulic state is not forcibly re-labeled as FAIL unless true well fault flags already exist.

## Mismatches found and what was restored

1. **RUN dry-stop incorrectly latched FAIL/alarm/block**
   - Restored to normal WAIT/recovery cycle.

2. **Link-loss forced well FAIL too aggressively**
   - Restored separation: safe stop outputs + link event logs, no forced hydraulic FAIL tagging.

3. **Pressure/filter warning role weakened by early dry FAIL**
   - Restored by removing dry-as-fail path so warning/block distinctions remain operationally meaningful.

4. **Stop logic invoked in fail context for normal dry event**
   - Reworked to call stop-accounting in non-fail semantic context.

## Transition semantics after restore

### Conditions that now transition to `WAIT`
- RUN dry condition (current below dry threshold for dry delay) after successful running.
- Non-blocking startup misses (before reaching max failed-start count).
- Link-loss/degraded fail-safe stop when no true hydraulic fault is active.

### Conditions that still transition/latch `FAIL`/`BLOCK`
- Emergency overcurrent.
- Sustained overload over delay.
- Pressure block threshold exceeded.
- Repeated failed starts reaching configured max.
- Existing explicit block flags (`wellBlocked`/`pressureBlock`).

## Communication-loss behavior now
- Telemetry is marked invalid when stale/lost.
- House VFD and well output are still forced safe (OFF).
- Well stop uses normal stop path (with accounting, no hydraulic fail pollution) when relay was ON.
- If already hydraulically blocked, state remains faulted by existing fault flags.

## Pressure/filter semantics now
- **Startup pressure validation**: still required in STARTING.
- **Pressure warning (`filterWarning`)**: visible warning-only state.
- **Pressure block (`pressureBlock`)**: hard protection with alarm + stop.
- Warning and block remain separate and observable in logs/UI.
