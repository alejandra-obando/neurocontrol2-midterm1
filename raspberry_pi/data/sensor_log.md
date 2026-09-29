# Sensor log

Generated: 2026-09-14 14:08:20  ·  samples seen: 250  ·  rows in the CSV: 70

## Diagnosis

- **ALERT** wheel DI: mean AGC 26 outside 80-220 -- AS5600 magnet off-center or at the wrong distance from the chip
- **ALERT** wheel DD: mean AGC 15 outside 80-220 -- AS5600 magnet off-center or at the wrong distance from the chip
- **ALERT** wheel TD: mean AGC 42 outside 80-220 -- AS5600 magnet off-center or at the wrong distance from the chip

## Wheels (moving window average)

| wheel | requested rpm | measured rpm | mean error | PWM | take-off | magnet AGC |
|---|---|---|---|---|---|---|
| DI | 0 | 0 | 0 | 0 | 95 | 26 |
| DD | 0 | 0 | 0 | 0 | 83 | 15 |
| TI | 0 | 0 | 0 | 0 | 93 | 97 |
| TD | 0 | -0 | 0 | 0 | 39 | 42 |

## ToF, in mm (chassis frame: N = front of the robot)

| sensor | minimum | mean | maximum | repeated readings in a row |
|---|---|---|---|---|
| N | 264 | 592 | 600 | 0 |
| E | 203 | 301 | 372 | 0 |
| S | 465 | 469 | 474 | 0 |
| O | 217 | 557 | 600 | 0 |

## Heading

- yaw: -0.5 to 0.0° (mean 0.0°)
- reference (yaw_ref): mean 0.0°
- heading correction: 0.0 to 0.0 rpm (mean 0.0)

## Session events

- no events yet

## Gains in force when this report was written

- kp = 0.25
- ki = 0.2
- kd = 0.0
- kpy = 3.6
- kiy = 0.0
- kdy = 0.7
- corrmax = 30.0
- signoyaw = 1.0
- crucero = 0.228
- vgiro = 60.0
- tolgiro = 3.5
- vgiromin = 25.0
- celda = 27.0
- odom = 1.0
