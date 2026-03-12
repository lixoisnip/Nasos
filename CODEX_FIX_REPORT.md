# CODEX Fix Report

## 1) I2C address mismatch found
- Root cause: Nano firmware was expected in hardware at address `0x09`, while ESP32 side attempted another address, causing `Wire.endTransmission()` error code `2` (address NACK / slave not responding).

## 2) Exact fix applied
- Unified Nano/ESP32 I2C address through shared config in `i2c_link_config.h`:
  - `i2cLinkCfg::NANO_SLAVE_ADDRESS = 0x09`.
- Kept both firmwares bound to that shared constant (`v1.ino` and `esp32_controller.ino` already reference it).
- Improved startup observability (already present and preserved):
  - Nano logs firmware name/version, configured slave address, and slave-mode init confirmation.
  - ESP32 logs SDA/SCL pins, expected Nano address, and startup probe result.
- Kept startup ACK probe on ESP32 before normal operation.

## 3) Analog divider recalibration performed
- Audited current conversion path in split architecture and updated house/VFD analog conversion in `v1.ino`.
- Replaced hidden formula with explicit calibration chain:
  1. raw ADC counts
  2. ADC node voltage
  3. divider gain restoration (from ADC node to original VFD output voltage)
  4. VFD voltage-to-current engineering conversion
- Added explicit clamp of reconstructed VFD voltage to `0..10V` before conversion to keep out-of-range ADC spikes from inflating engineering current.
- Added explicit constants in `v1.ino`:
  - `ADC_REFERENCE_V`
  - `ADC_MAX_COUNTS`
  - `VFD_DIVIDER_R_TOP_OHM`
  - `VFD_DIVIDER_R_BOTTOM_OHM`
  - `VFD_DIVIDER_GAIN`
  - `VFD_OUTPUT_MAX_V`
  - `VFD_OUTPUT_MAX_CURRENT_A`
  - `VFD_ANALOG_V_TO_CURRENT_A`
- Calibration assumes new divider `20k/10k` and engineering mapping `0–10V = 0–9A`.

## 4) Protection thresholds checked
- Verified that well/house protection thresholds are still expressed in physical units (A/bar), and kept unchanged.
- House pump thresholds (`dry`, `overload`, `emergency`) continue to compare against amperes after recalibrated analog conversion.
- Failsafe behavior on Nano link degradation/loss remains intact (no weakening/removal).

## 5) Assumptions used
- VFD analog output represents `0–10V` proportional to `0–9A`.
- Divider is now `20k (top) + 10k (bottom)` for safe ADC node level (~3.33V at 10V input).
