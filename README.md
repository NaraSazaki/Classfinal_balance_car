# Balance Car

STM32F103C8T6 balance car projects for self-balancing, TCRT5000 line tracking, and ramp climbing.

## Projects

- `control`: Base self-balancing program. This is the stable balance-control reference used by later projects.
- `go side`: Balance car with successful TCRT5000 line tracking logic.
- `climb-ramp`: Current integrated version for balancing, line tracking, and ramp climbing.
- `climb-ramp_good`: Backup version saved while improving `climb-ramp`; kept as a more stable reference build.

## Hardware Pins

| Module | Pin |
| --- | --- |
| PWMA | PA0 |
| PWMB | PA1 |
| AIN1 | PA4 |
| AIN2 | PA5 |
| BIN1 | PA6 |
| BIN2 | PA7 |
| MPU6050 SCL | PB6 |
| MPU6050 SDA | PB7 |
| USART1_TX/RX | PA9 / PA10 |
| TCRT5000 D0 | PA2 / PB1 |
| USB_DM/DP | PA11 / PA12 |

## Development Environment

- MCU: STM32F103C8T6
- IDE/tooling: Keil uVision5, STM32CubeMX
- Main modules:
  - MPU6050 IMU
  - TB6612FNG motor driver
  - TCRT5000 tracking sensors
  - USB CDC / UART debug support

## Repository Layout

```text
balance-car/
|-- control/
|   |-- Core/
|   |-- Drivers/
|   |-- Middlewares/
|   |-- USB_DEVICE/
|   |-- MDK-ARM/
|   `-- control.ioc
|
|-- go side/
|   |-- Core/
|   |-- Drivers/
|   |-- Middlewares/
|   |-- USB_DEVICE/
|   |-- MDK-ARM/
|   `-- control.ioc
|
|-- climb-ramp/
|   |-- Core/
|   |-- Drivers/
|   |-- Middlewares/
|   |-- USB_DEVICE/
|   |-- MDK-ARM/
|   `-- control.ioc
|
`-- climb-ramp_good/
    |-- Core/
    |-- Drivers/
    |-- Middlewares/
    |-- USB_DEVICE/
    |-- MDK-ARM/
    `-- control.ioc
```

## Current `climb-ramp` Update

The current `climb-ramp` version combines the improved balance base, successful line tracking, and ramp-climbing behavior.

### Balance

- Uses the improved `control` balance logic.
- Uses MPU6050 angle estimation with gyro-based derivative control.
- Includes integral limiting, output limiting, PWM slew limiting, and fall protection.

### Line Tracking

- Uses the successful TCRT5000 logic refined from `go side`.
- Sensor convention:
  - LED on means white surface.
  - LED off means black line.
- One-black/one-white correction remains active on flat ground and while climbing.
- Short line-loss hold keeps the last correction briefly when the car bounces and the sensor temporarily loses the line.

### Ramp Climbing

- Adds ramp-climb mode when the body angle reaches the ramp region.
- Flat-ground tracking keeps the slower, stable behavior.
- Ramp mode raises motor output only when climbing.
- Straight ramp climbing can use fixed equal wheel output.
- Ramp tracking still applies one-black/one-white correction while keeping higher climb power.
- During ramp correction, the slower wheel is kept at a small nonzero speed to reduce one-sided slipping or free spinning.
- Ramp mode exits after the body angle returns near flat ground.

## Build

1. Open the desired project in Keil uVision5.
2. Use the project file under `MDK-ARM`, for example:

```text
climb-ramp/MDK-ARM/control.uvprojx
```

3. Select/build for STM32F103C8T6.
4. Flash with ST-Link.

Generated Keil build outputs such as `.o`, `.d`, `.axf`, `.hex`, `.map`, and build logs are ignored by Git.

## Notes

- Keep changes inside the target project folder when testing a behavior.
- `climb-ramp_good` is kept as a backup reference before more aggressive ramp changes.
- If STM32CubeMX regenerates code, check `USER CODE BEGIN` / `USER CODE END` regions before committing.
