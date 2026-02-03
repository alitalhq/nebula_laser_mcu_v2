# Nebula Laser MCU v2

A precision laser gimbal control system for stabilizing and pointing a laser on aerial platforms (drones). Built on ESP32-S3 with real-time dual-core processing.

## Overview

The Nebula Laser MCU v2 provides:
- **Real-time stabilization** against drone vibrations and body movements
- **Dual-mode operation**: Ground-lock (gravity-referenced) or ROS2-commanded tracking
- **Redundant sensing**: Dual IMUs + dual magnetic encoders
- **Low latency control**: 1000 Hz IMU reading, 500 Hz stabilization loop

## Hardware Requirements

### Microcontroller
- ESP32-S3 (dual-core 240MHz with PSRAM)

### Sensors
| Sensor | Model | Interface | Purpose |
|--------|-------|-----------|---------|
| Body IMU | BMI160 | I2C0 (0x68) | Drone body orientation |
| Head IMU | BMI160 | I2C1 (0x68) | Gimbal head orientation |
| Pan Encoder | AS5600 | Soft I2C | Pan axis position |
| Tilt Encoder | AS5600 | Soft I2C | Tilt axis position |

### Actuators
| Component | GPIO | Description |
|-----------|------|-------------|
| Pan Motor Step | 1 | Stepper motor step signal |
| Pan Motor Dir | 2 | Stepper motor direction |
| Tilt Motor Step | 4 | Stepper motor step signal |
| Tilt Motor Dir | 5 | Stepper motor direction |
| Motor Enable | 6 | Shared enable (active low) |
| Laser | 7 | Laser on/off control |
| Buzzer | 39 | Audio feedback (PWM) |

### Motor Configuration
- Steps per revolution: 200
- Microstepping: 32x
- Effective resolution: 6400 steps/revolution

### Movement Limits
- Pan: -30° to +30°
- Tilt: -20° to +20°

## Software Architecture

### FreeRTOS Tasks

| Task | Frequency | Core | Priority | Purpose |
|------|-----------|------|----------|---------|
| IMUReadTask | 1000 Hz | 0 | 6 | Parallel IMU reading |
| StabilizationTask | 500 Hz | 0 | 5 | Body motion compensation |
| PositionControlTask | 200 Hz | 0 | 4 | Target tracking |
| SerialTask | Async | 1 | 3 | ROS2 communication |
| DiagnosticsTask | 10 Hz | 1 | 1 | System health monitoring |

### Module Structure

```
src/
├── main.cpp                    # Entry point
├── config/
│   ├── HardwareConfig.h        # Pin assignments, I2C addresses
│   └── ControlConfig.h         # PID gains, limits
├── hardware/
│   ├── IMUDriver.*             # BMI160 driver
│   ├── EncoderDriver.*         # AS5600 driver
│   ├── StepperTimer.*          # Motor pulse generation
│   ├── BuzzerDriver.*          # Audio feedback
│   └── SoftI2C.*               # Software I2C
├── sensors/
│   ├── IMUFusion.*             # Complementary filter
│   ├── EncoderFilter.*         # Median filtering
│   ├── SensorCalibration.*     # NVS calibration storage
│   └── SensorHealth.*          # Error tracking
├── control/
│   ├── PositionController.*    # PI position servo
│   ├── StabilizationController.* # Feed-forward stabilization
│   ├── CommandCombiner.*       # Velocity merging & limits
│   ├── LimitEnforcer.*         # Mechanical boundaries
│   └── GroundLockController.*  # Gravity-reference mode
├── communication/
│   ├── SerialProtocol.*        # Binary protocol
│   ├── TargetManager.*         # Mode management
│   └── ParallelIMUReader.*     # Dual-IMU sync
├── tasks/
│   └── Tasks.*                 # FreeRTOS task implementations
└── utils/
    ├── AtomicData.h            # Thread-safe data wrapper
    ├── MathUtils.*             # Angle math utilities
    └── RingBuffer.h            # Circular buffer
```

## Communication Protocol

Serial communication runs at **921600 baud** using a custom binary protocol.

### Command Packet (22 bytes)
| Field | Type | Description |
|-------|------|-------------|
| Header | uint16 | 0xAA55 |
| Pan Delta | float | Target pan offset (degrees) |
| Tilt Delta | float | Target tilt offset (degrees) |
| Pan FF Velocity | float | Feed-forward pan velocity (°/s) |
| Tilt FF Velocity | float | Feed-forward tilt velocity (°/s) |
| Laser Control | uint8 | 0=off, 1=on |
| CRC16 | uint16 | Checksum |

### Telemetry Packet (32 bytes)
| Field | Type | Description |
|-------|------|-------------|
| Header | uint16 | 0x55AA |
| Current Pan | float | Current pan position (degrees) |
| Current Tilt | float | Current tilt position (degrees) |
| Pan Error | float | Position error (degrees) |
| Tilt Error | float | Position error (degrees) |
| Status Flags | uint16 | System state flags |
| CRC16 | uint16 | Checksum |

## Building

### Prerequisites
- [PlatformIO](https://platformio.org/) (CLI or VSCode extension)

### Build Commands

```bash
# Build
pio run -e esp32s3

# Upload
pio run -e esp32s3 -t upload

# Monitor serial output
pio device monitor -b 921600
```

## Startup Sequence

1. Hardware initialization (I2C buses, GPIO, sensors)
2. Gravity reference calibration (drone must be on level ground)
3. Gyroscope calibration (~10 seconds, keep drone still)
4. Load stored calibration from NVS
5. Initialize control loops
6. Start FreeRTOS tasks
7. Enable watchdog timer (3-second timeout)
8. Enable stepper motors
9. System ready (3 beeps)

## Control Modes

### Ground Lock Mode
- Active when no ROS2 commands received
- Points laser downward using gravity reference
- Automatic fallback after communication timeout (2-5 seconds)

### Tracking Mode
- Active when receiving valid ROS2 commands
- Follows commanded pan/tilt positions
- Combines position control with stabilization

## Control Parameters

### Position Controller (PI)
| Parameter | Pan | Tilt |
|-----------|-----|------|
| Kp | Configurable | Configurable |
| Ki | Configurable | Configurable |
| Dead Zone | 0.1° | 0.1° |

### Velocity Limits
| Axis | Max Velocity | Max Acceleration |
|------|--------------|------------------|
| Pan | 100 °/s | 500 °/s² |
| Tilt | 80 °/s | 400 °/s² |

### Stabilization
- Gyroscope filter cutoff: 50 Hz
- Complementary filter alpha: 0.98

## Safety Features

- **Watchdog Timer**: Automatic reset if system hangs (3-second timeout)
- **Communication Timeout**: Falls back to ground-lock mode
- **Mechanical Limits**: Hard stops with soft margin (5°) for gradual slowdown
- **Sensor Health Monitoring**: Tracks error rates and consecutive failures
- **Motor Enable Control**: Can be disabled in emergency

## Calibration

Calibration data is stored in ESP32 NVS (Non-Volatile Storage):
- Gyroscope bias offsets (both IMUs)
- Mechanical limit positions
- Gravity reference vectors

To recalibrate:
1. Power on with drone on level surface
2. Keep drone still during gyro calibration (~10 seconds)
3. Calibration is automatically saved

## License

This project is licensed under the Apache License, Version 2.0.  
See the [LICENSE](LICENSE) file for details.
