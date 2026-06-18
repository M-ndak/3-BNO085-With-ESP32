# 3x BNO085 Simultaneous I2C on ESP32

Running three BNO085 IMU sensors concurrently on a single ESP32 over I²C,
using a custom multi-sensor patch of the SparkFun BNO08x library that moves
all file-scope globals into per-instance contexts.

## The Problem This Solves

The stock SparkFun BNO08x library uses file-scope globals, meaning only one
sensor can operate at a time. This project uses a patched version that gives
each sensor its own isolated context, allowing up to 3 sensors to run
simultaneously without interfering with each other.

## Hardware

- ESP32
- 3x BNO085 IMU sensors
- 4.7kΩ pull-up resistors on SDA and SCL

## I2C Addressing

The BNO085 only has two possible I2C addresses, so the third sensor is handled
by multiplexing:

| Sensor | ADR Pin | Address |
|--------|---------|---------|
| IMU 1  | GND     | 0x4A    |
| IMU 2  | 3.3V    | 0x4B    |
| IMU 3  | Multiplexed | 0x4A / 0x4B |

## Wiring

| BNO085 Pin | ESP32 Pin          |
|------------|--------------------|
| VDD        | 3.3V               |
| GND        | GND                |
| SDA        | GPIO21 + 4.7kΩ pullup |
| SCL        | GPIO22 + 4.7kΩ pullup |
| RST        | GPIO4 (optional)   |
| PS0        | GND (I²C mode)     |
| PS1        | GND (I²C mode)     |

## Output Format

IMU0  roll:  12.34  pitch:   5.67  yaw:  89.01	IMU1  ax:  0.12  ay:  0.34  az:  9.81	IMU2:(waiting)
IMU0  ax:  0.12  ay: -0.34  az:  9.81	        IMU1  roll:   1.23  pitch:  4.56  yaw:  78.90	IMU2  gx: 0.001  gy: 0.002  gz: 0.000

Reported at 100Hz (10ms interval) over Serial at 115200 baud.

## Dependencies

- [BNO085-ESP32-Library](https://github.com/M-ndak/BNO085-ESP32-Library-) — custom multi-sensor patch (included as submodule)

## Setup

1. Clone with submodules:
```bash
git clone --recurse-submodules git@github.com:M-ndak/3-BNO085-With-ESP32.git
```
2. Open `firmware/tracker_imu/tracker_imu.ino` in Arduino IDE
3. Install ESP32 board support if not already installed
4. Select your ESP32 board and port
5. Flash

## Known Issues

- Occasional sensor dropout at startup — power cycle resolves it

## Dev Log

See [DEVLOG.md](DEVLOG.md) for development history.
