# speedometer_vingage — hardware overview & pinout

## Platform
- **MCU board:** Arduino Nano
- **Display:** I2C OLED, 128×32 (SSD1306 driver, via `Adafruit_SSD1306` / `Adafruit_GFX`)
- **Speedometer needle:** X27.168 stepper gauge motor (Switec/Escap-family), 2-phase bipolar, 315° max sweep — confirmed by `STEPS_MAX = 630` matching `DEG10_TO_STEPS(3150°·10)`
- **Speed scale linearization:** LUT (`speedAngleLUT`, 15 points, 10 km/h step) mapping speed → needle angle, to compensate for the gauge face's non-linear speedometer markings
- **Distance persistence:** EEPROM wear-leveling — distance is written to one of `RECORD_COUNT = 20` rotating slots (`currentRecordIndex`), each record tagged with a sequence number and CRC-8, so writes are spread across EEPROM cells and the most recent valid record is recovered on boot (`loadDistance()`)

## Power input
- 5V is fed into the Arduino Nano's 5V pin through a **0.5 W series resistor**.
- A **unidirectional TVS diode, 15 V, cathode up (toward 5V)** is connected between the Nano's 5V pin and ground, for transient/overvoltage protection on the automotive supply line.

## Pinout

| Pin | Direction | Code reference | Purpose |
|---|---|---|---|
| **A0** | OUTPUT | `STEP_PINS[0]` | X27.168 pin 1 — Coil A, terminal 1 |
| **A1** | OUTPUT | `STEP_PINS[1]` | X27.168 pin 2 — Coil B, terminal 1 |
| **A2** | OUTPUT | `STEP_PINS[2]` | X27.168 pin 3 — Coil A, terminal 2 |
| **A3** | OUTPUT | `STEP_PINS[3]` | X27.168 pin 4 — Coil B, terminal 2 |
| **A4 (SDA)** | I2C | implicit, via `Wire` | I2C data line for the 128×32 OLED display (SSD1306) |
| **A5 (SCL)** | I2C | implicit, via `Wire` | I2C clock line for the OLED display |
| **D1** | INPUT, PCINT (PCMSK2 bit1) | `rawMask` bit0 | Gear selector input — only used combined with D4 (state `'3'`) |
| **D2** | INPUT, PCINT (PCMSK2 bit2) | `rawMask` bit1 | Gear selector: state `'1'` |
| **D3** | INPUT, PCINT (PCMSK2 bit3) | `rawMask` bit2 | Gear selector: state `'2'` |
| **D4** | INPUT, PCINT (PCMSK2 bit4) | `rawMask` bit3 | Gear selector: state `'D'`, and combined with D1 → `'3'` |
| **D5** | INPUT, PCINT (PCMSK2 bit5) | `rawMask` bit4 | Gear selector: state `'N'` |
| **D6** | INPUT, PCINT (PCMSK2 bit6) | `rawMask` bit5 | Gear selector: state `'R'` |
| **D7** | INPUT, PCINT (PCMSK2 bit7) | `rawMask` bit6 | Gear selector: state `'P'` |
| **D8 (PB0)** | INPUT, PCINT0 | `SENSOR_PIN` | Speed/odometer pulse sensor input (feeds `pulseCounter`, `speedPulses`) |
| **D0** | unused | — | PCMSK2 bit0 not enabled (`0b11111110`), pin is free |

## Notes

- A0–A3 drive the 4 leads of the X27.168's 2 bipolar coils (A0/A2 = coil A, A1/A3 = coil B, per the datasheet pinout — assumes `STEP_PINS[0..3]` are wired to motor pins 1→4 in order). The `fullStep` table here is a **wave-drive** sequence (one pin active at a time), whereas the motor's "official" driving scheme normally energizes two pins at once per step for full torque — this firmware trades torque for a simpler single-transistor-per-lead driver stage.
- Timers 1 and 2 (`TIMER1_COMPA`, `TIMER2_COMPA`) are used only as internal interrupt sources (CTC mode); they're never routed to their output-compare pins (OC1A/OC2A = D9/D11), so those pins remain physically free.
- D1–D7 (gear selector inputs) never get a `pinMode()` call — they're left in the default `INPUT` state with no internal pull-up. External optocoupler matching device used. Without external pull-ups/pull-downs (or a driving external circuit), these inputs are effectively floating.
