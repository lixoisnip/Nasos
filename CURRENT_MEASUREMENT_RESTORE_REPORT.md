# Current Measurement Restore Report

## Why Modbus current was removed
The active hardware does not provide reliable house pump current feedback via VFD Modbus registers. Using `REG_CURRENT_FEEDBACK` as house current source produced a wrong measurement path. The ESP32 now uses Modbus only for VFD control/status operations (run/stop/frequency/run-state poll), not for house current feedback.

## Nano telemetry source used
House pump current input is restored to Nano analog telemetry:
- Nano reads house current analog channel from `PIN_CURRENT` (`A0`), averaging multiple ADC samples.
- Nano sends this value in telemetry field `analogAuxRaw`.
- ESP32 consumes `payload.analogAuxRaw` as the only source for house current.

## Conversion formula on ESP32
ESP32 now performs explicit conversion in `convertNanoAnalogToCurrent(uint16_t raw)`:
1. `adc_voltage = raw * NANO_ADC_VREF / NANO_ADC_MAX`
2. `source_voltage = adc_voltage * CURRENT_SENSOR_DIVIDER_RATIO`
3. `current = source_voltage * CURRENT_SENSOR_MAX_CURRENT / CURRENT_SENSOR_MAX_VOLTAGE`
4. `current = current * CURRENT_CALIBRATION_GAIN + CURRENT_CALIBRATION_OFFSET`
5. Clamp to sane range and apply near-zero suppression.

## Divider assumptions and calibration constants
Configurable constants are centralized in `houseCurrentSense` namespace:
- `NANO_ADC_MAX`
- `NANO_ADC_VREF`
- `CURRENT_SENSOR_DIVIDER_RATIO`
- `CURRENT_SENSOR_MAX_VOLTAGE`
- `CURRENT_SENSOR_MAX_CURRENT`
- `CURRENT_CALIBRATION_GAIN`
- `CURRENT_CALIBRATION_OFFSET`
- `NEAR_ZERO_CLAMP_A`

This keeps divider tuning explicit and avoids hidden scaling assumptions.

## Filtering strategy
Filtering is applied in two stages:
- **Nano side:** multi-sample averaging for `PIN_CURRENT` (`HOUSE_CURRENT_AVG_SAMPLES`).
- **ESP32 side:** dual exponential smoothing:
  - faster `houseCurrentProtection` for protection logic,
  - slower `houseCurrentDisplay` for stable UI.

Both values apply near-zero clamp to suppress false tiny readings.

## Protections checked
House protections still execute in amperes using the restored measurement chain:
- emergency overload
- overload delay protection
- dry-run by current
- no-current/low-current startup behavior (through existing startup ignore + dry logic)
- fault/alarm and auto-restart transitions

Threshold semantics are preserved (`cfg.house.*Current` in amps); only the measurement source and quality were corrected.
