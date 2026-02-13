# Wiring Reference

## Arduino Nano Pin Map

### TMC2209
- `D2 -> STEP`
- `D3 -> DIR`
- `D4 -> EN/ENN` (active-low enable)
- `5V -> VIO`
- `GND -> GND`

### HC-SR04
- `D8 -> TRIG`
- `D9 -> ECHO`
- `5V -> VCC`
- `GND -> GND`

### AS5600
- `A4 -> SDA`
- `A5 -> SCL`
- `5V -> VCC/VIN`
- `GND -> GND`
- I2C address: `0x36`

## Power and Ground

- Driver motor supply: `12V + -> VMOT/VM`, `12V - -> common GND`.
- Common ground must include Nano, TMC2209, AS5600, HC-SR04, and 12V negative.
- Place `100uF` electrolytic capacitor near `VMOT-GND` on driver.
- Keep `EN/ENN` pulled up to `VIO` through `10k` resistor.

## Microstepping

Current default from hardware setup: `1/16 microstep`.
