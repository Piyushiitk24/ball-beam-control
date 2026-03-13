# Wiring Reference

## Arduino Nano Pin Map

### TMC2209
- `D2 -> STEP`
- `D3 -> DIR`
- `D4 -> EN/ENN` (active-low enable)
- `5V -> VIO`
- `GND -> GND`

### Sharp GP2Y0A21YK0F (active ball-position sensor)
- `A0 -> VO`
- `5V -> VCC`
- `GND -> GND`

### Archived HC-SR04 / diagnostics only
- `D8 -> TRIG`
- `D9 -> ECHO`
- `5V -> VCC`
- `GND -> GND`

(Legacy: TFMini D10/D11 — not connected.)

### AS5600
- `A4 -> SDA`
- `A5 -> SCL`
- `5V -> VCC/VIN`
- `GND -> GND`
- I2C address: `0x36`

## Power and Ground

- Driver motor supply: `12V + -> VMOT/VM`, `12V - -> common GND`.
- Common ground must include Nano, TMC2209, AS5600, Sharp sensor, and 12V negative.
- Place `100uF` electrolytic capacitor near `VMOT-GND` on driver.
- Keep `EN/ENN` pulled up to `VIO` through `10k` resistor.

## Microstepping

Current default from hardware setup: `1/16 microstep`.
