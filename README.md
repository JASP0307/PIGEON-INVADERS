# PIGEON-INVADERS

An IoT embedded system for automated pigeon deterrence, built on an ESP32-S3 microcontroller with a camera and laser-guided servo gimbal.

## Overview

PIGEON-INVADERS monitors a physical area using a camera mounted on a 2-axis servo gimbal (pan/tilt). When a pigeon is detected, the system activates a laser pointer and moves it in zigzag patterns across calibrated zones to scare the bird away. The operator controls everything remotely via a Telegram bot -- starting/stopping monitoring, calibrating zones, adjusting schedules, and receiving pre/post-attack photos.

The system runs 3 concurrent FreeRTOS tasks across the ESP32-S3's dual cores: Telegram polling on Core 0, and a finite state machine plus servo control on Core 1.

---

## Project Structure

```
PIGEON-INVADERS/
|
|-- platformio.ini                        # PlatformIO build configuration
|-- README.md                             # This file
|-- README_PICS.md                        # Documentation for dataset capture sketch (Spanish)
|-- esp32s3_captura_automatica.ino        # Standalone sketch for dataset image collection
|
|-- include/                              # Header files
|   |-- IO_Points.h                       # Hardware GPIO pin assignments
|   |-- SystemDefinitions.h               # Central definitions: FSM states, events, structs, externs
|   |-- credentials.h                     # Extern declarations for WiFi/Telegram credentials (gitignored)
|   |-- ServoModule.h                     # ServoModule class declaration
|   |-- Laser.h                           # Laser class declaration
|   |-- servo_patterns.h                  # Servo movement pattern API
|   |-- camera_manager.h                  # Camera init/capture/send API
|   |-- telegram_bot.h                    # Telegram bot API
|   |-- wifi_manager.h                    # WiFi connection management API
|   |-- calibration.h                     # NVS calibration persistence API
|   |-- schedule.h                        # Operating hours / schedule API
|
|-- src/                                  # Source files
|   |-- main.cpp                          # Entry point, FSM, servo task, glue logic
|   |-- ServoModule.cpp                   # Servo class: smooth single-degree movement
|   |-- Laser.cpp                         # Laser class: on/off/toggle via GPIO
|   |-- servo_patterns.cpp                # Zigzag and rectangle movement patterns
|   |-- camera_manager.cpp                # OV2640 camera init, capture, Telegram photo upload
|   |-- telegram_bot.cpp                  # Telegram bot: polling, command parsing, inline keyboards
|   |-- wifi_manager.cpp                  # WiFi connect, switch, reconnect
|   |-- calibration.cpp                   # Load/save calibration data to NVS
|   |-- schedule.cpp                      # Load schedules, check operating hours
|
|-- Dataset/                              # Dataset preparation tools
|   |-- imagecrop.py                      # Python/OpenCV script to crop pigeon images
|
|-- test/                                 # PlatformIO test directory (no tests written)
|-- lib/                                  # PlatformIO private libraries directory
|-- .vscode/                              # VS Code configuration (auto-generated)
|-- .pio/                                 # PlatformIO build artifacts (generated)
```

---

## File Descriptions

### Build & Configuration

| File | Description |
|------|-------------|
| `platformio.ini` | Master build configuration. Defines the board (`esp32s3usbotg`), framework (Arduino), library dependencies (`UniversalTelegramBot`, `ESP32Servo`, `ArduinoJson`), build flags (PSRAM support), flash size (16MB), and monitor speed (115200 baud). |
| `.gitignore` | Excludes `.pio/` build artifacts, VS Code internal files, and `credentials.h` from version control. |

### Header Files (`include/`)

| File | Description |
|------|-------------|
| `IO_Points.h` | Defines hardware GPIO pin assignments using namespaces: `Pinout::ServoMotors::SERVO_X` (GPIO 42), `Pinout::ServoMotors::SERVO_Y` (GPIO 21), `Pinout::Laser::Laser_1` (GPIO 48). |
| `SystemDefinitions.h` | Central header defining camera GPIO pins for the Freenove ESP32-S3 CAM, FSM states (`SystemState` enum: INITIALIZING, IDLE, MONITORING, ATTACKING, calibration states, etc.), FSM events (16 event types), pattern types (RECTANGLE_PREVIEW, ZIGZAG_HORIZ, ZIGZAG_VERT), data structures (`PatternContext`, `ManualPosCmd`, `RangoHorario`), and all `extern` declarations for global variables, FreeRTOS handles, and hardware objects. |
| `credentials.h` | Extern declarations for WiFi SSIDs/passwords and Telegram bot token/chat ID. The actual values are defined in `main.cpp`. This file is gitignored for security. |
| `ServoModule.h` | Declares the `ServoModule` class: constructor (pin), `begin()`, `setTarget()` (with lazy servo attach), `update()` (single-degree step), `getPosition()`, `getTarget()`. |
| `Laser.h` | Declares the `Laser` class: `on()`, `off()`, `toggle()`, `isOn()`. |
| `servo_patterns.h` | Declares the pattern API: `startPattern()`, `stopPattern()`, `updatePatternLogic()`. Defines zigzag step size (1.0) and target tolerance (0.1). |
| `camera_manager.h` | Declares camera API: `camera_init()`, `camera_capture()`, `camera_releaseFrame()`, `camera_sendToTelegram()`. |
| `telegram_bot.h` | Declares Telegram API: `telegram_init()`, `telegram_task()`, `telegram_sendMessage()`, `telegram_sendMenu()`. |
| `wifi_manager.h` | Declares WiFi API: `wifi_connect()`, `wifi_switch()`, `wifi_reconnect()`, `wifi_isConnected()`. |
| `calibration.h` | Declares NVS persistence API: `guardarCalibracion()`, `cargarCalibraciones()`, `cargarVelocidades()`, `guardarPosicionServos()`, `cargarPosicionServosX/Y()`. |
| `schedule.h` | Declares schedule API: `cargarHorarios()`, `isWithinOperatingHours()`. |

### Source Files (`src/`)

| File | Description |
|------|-------------|
| `main.cpp` | System entry point and glue layer. Defines global variables (credentials, calibration arrays, FSM state, FreeRTOS handles, hardware objects), `setup()` (creates mutex, queues, and spawns 3 FreeRTOS tasks), `loop()` (empty -- all work done in tasks), `TaskFSM` (main finite state machine on Core 1, processes events from a queue and transitions between states), `TaskServoControl` (runs on Core 1, executes movement patterns or manual servo commands, drives physical servo movement), all `onEnter*()` callbacks for each FSM state, timer callbacks for monitoring interval and idle schedule checking, and helper `capturarYEnviarFoto()` for capturing and sending photos with timestamps. |
| `ServoModule.cpp` | Implements the `ServoModule` class. Servos are lazily attached -- the PWM signal is only applied when the servo needs to move (`setTarget()`), and servo writes are done one degree at a time in `update()` for smooth movement. The servo detaches when idle to reduce jitter and power consumption. |
| `Laser.cpp` | Implements the `Laser` class. Simple GPIO digital output control with `on()`/`off()`/`toggle()` methods and internal state tracking. |
| `servo_patterns.cpp` | Implements laser movement patterns. Three pattern types: `PATTERN_RECTANGLE_PREVIEW` (traces four corners of a calibration area), `PATTERN_ZIGZAG_HORIZ` (horizontal sweeps with 1-degree steps across Y while alternating X), `PATTERN_ZIGZAG_VERT` (vertical sweeps with 1-degree steps across X while alternating Y). Supports multi-area attack sequences. |
| `camera_manager.cpp` | Implements the camera subsystem. Initializes OV2640 with Freenove ESP32-S3 pin mapping, VGA resolution with PSRAM, double buffering. `camera_sendToTelegram()` manually constructs an HTTPS multipart/form-data POST to the Telegram Bot API, chunking the JPEG in 2KB blocks. |
| `telegram_bot.cpp` | The most feature-rich file. Implements the full Telegram bot interface as a FreeRTOS task polling for messages every 1 second. Supports inline keyboard menu with 14 command buttons, calibration arrow keyboard for manual servo positioning, and text commands: START, STOP, FOTO, SIMULAR, CALIB 1-4, VER 1-4, CLARO, FAMILIA, HORARIOS, CONFIG, STATUS, HELP, /horario, /mins, /speed. Only responds to one authorized chat ID. All commands translate to FSM events sent via `xQueueSend()`. |
| `wifi_manager.cpp` | Implements WiFi management. `wifi_connect()` is blocking with a 30-second timeout (restarts ESP32 on failure). `wifi_switch()` disconnects, tries the new network, and automatically rolls back to the previous network on failure. `wifi_reconnect()` handles automatic reconnection. |
| `calibration.cpp` | Implements NVS (Non-Volatile Storage) persistence. Saves/loads calibration bounds (minX, maxX, minY, maxY) and home positions for all 4 areas using separate NVS namespaces (`calib0`-`calib3`). Also saves/loads servo speeds and monitoring interval from `config` namespace, and last servo position from `servoPos` namespace. |
| `schedule.cpp` | Implements operating hours. Loads up to 3 time windows from NVS (default: 07:00-18:00 for schedule 1). `isWithinOperatingHours()` converts current NTP time to minutes-since-midnight and checks if it falls within any active schedule, handling wrap-around for overnight schedules. |

### Dataset Tools

| File | Description |
|------|-------------|
| `esp32s3_captura_automatica.ino` | Standalone Arduino sketch (not part of the PlatformIO project) for collecting a training dataset. Two modes: fixed interval capture, or motion detection via frame differencing. Saves JPEG images to a microSD card under `/dataset/`. |
| `Dataset/imagecrop.py` | Python script using OpenCV to batch-crop a region of interest (280x280 pixels starting at x=106, y=79) from all `.jpg` files in a source folder and save them to a destination folder. Used to preprocess captured images for model training. |

---

## Tech Stack

| Layer | Technology |
|-------|------------|
| Microcontroller | ESP32-S3 (Freenove ESP32-S3 WROOM CAM board) |
| Framework | Arduino (via ESP-IDF) |
| Build System | PlatformIO |
| Language | C++ (firmware), Python (dataset tooling) |
| RTOS | FreeRTOS (dual-core: Telegram on Core 0, FSM + Servo on Core 1) |
| Camera | OV2640 via `esp_camera` driver (VGA resolution, JPEG, double-buffered with PSRAM) |
| Actuators | 2x Servo motors (pan/tilt), 1x Laser diode |
| Persistence | ESP32 NVS (Preferences library) |
| Connectivity | WiFi (dual-network support with automatic rollback) |
| Communication | Telegram Bot API (HTTPS, multipart photo upload, inline keyboards) |
| Libraries | UniversalTelegramBot v1.3.0, ESP32Servo v3.2.1, ArduinoJson v7.2.2 |
| Time Sync | NTP (UTC-4 / Atlantic timezone) |
| Data Processing | OpenCV (Python, for image cropping) |

---

## System Architecture

### Task Architecture

```
Core 0:  [telegram_task]         -- Polls Telegram for commands, sends events to FSM

Core 1:  [TaskFSM]               -- Processes events, manages state transitions
         [TaskServoControl]      -- Drives servos, executes movement patterns
         [FreeRTOS Timers]       -- monitoringTimer (2-min interval)
                                   updateIdleLogicTimer (1-min schedule check)
```

### Communication Primitives

- **`fsmQueue`** -- Queue (depth 10, `FSMEvent`): All tasks send events here; the FSM task reads from it.
- **`manualControlQueue`** -- Queue (depth 5, `ManualPosCmd`): Telegram task sends manual servo commands; the Servo task reads from it.
- **`stateMutex`** -- Mutex: Protects `currentState` for thread-safe read/write across tasks.
- **`monitoringTimer`** / **`updateIdleLogicTimer`** -- Software timers that generate `EVENT_TIMER_EXPIRED` and `EVENT_START/STOP_COMMAND` events.

### Finite State Machine (FSM)

```
STATE_INITIALIZING
      |
      | EVENT_INIT_COMPLETE (WiFi, camera, NTP, Telegram ready)
      v
STATE_IDLE  <---------------------------------------------+
      |                                                     |
      | EVENT_START_COMMAND (manual or auto-schedule)       | EVENT_STOP_COMMAND
      v                                                     |
STATE_MONITORING ---+                                       |
      |             |                                       |
      | EVENT_PIGEON_DETECTED /                             |
      | EVENT_MANUAL_COMMAND /                              |
      | EVENT_NO_PIGEON / EVENT_TIMER_EXPIRED               |
      v                                                     |
STATE_PICTURE_PRE_ATTACK                                    |
      |                                                     |
      | EVENT_PROCESSING_COMPLETE (photo sent)              |
      v                                                     |
STATE_ATTACKING                                             |
      |                                                     |
      | EVENT_ATTACK_COMPLETE                               |
      v                                                     |
STATE_PICTURE_POST_ATTACK                                   |
      |                                                     |
      | EVENT_PROCESSING_COMPLETE (photo sent)              |
      v                                                     |
      +---------> back to STATE_MONITORING --+--------------+
                                             |
Calibration Flow:                            |
IDLE -> STATE_CALIB_SET_LL -> STATE_CALIB_SET_UR
      -> STATE_CALIB_SAVE -> STATE_CALIB_PREVIEW -> IDLE
```

### Movement Patterns

The system implements three laser movement patterns:

- **Rectangle Preview** -- Traces the four corners of a calibrated area (one shot).
- **Horizontal Zigzag** -- Sweeps left-right while incrementing Y by 1 degree per step. When Y exceeds the max, moves to the next area in the attack sequence.
- **Vertical Zigzag** -- Sweeps up-down while incrementing X by 1 degree per step. Same area sequencing logic.

Patterns support multi-area attack sequences: when a pigeon is detected, the system attacks the detected area first, then sweeps through all remaining 3 areas.

### NVS Persistence

| Namespace | Keys | Purpose |
|-----------|------|---------|
| `calib0`..`calib3` | minX, maxX, minY, maxY, homeX, homeY | Calibration bounds for each of 4 areas |
| `config` | intervalMins, Speed, st0..st2, en0..en2 | Monitoring interval, servo speed, 3 schedule windows |
| `servoPos` | lastX, lastY | Last known servo position (restored on boot) |

---

## Getting Started

### Prerequisites

1. Install [PlatformIO](https://platformio.org/) (CLI or VS Code extension).
2. Have a Freenove ESP32-S3 CAM board connected via USB.

### Build

```bash
pio run
```

Compiles the project for the `esp32s3usbotg` environment. Output: `.pio/build/esp32s3usbotg/firmware.bin`.

### Flash

```bash
pio run --target upload
```

### Monitor Serial Output

```bash
pio device monitor --baud 115200
```

### First-Time Setup

1. Create `include/credentials.h` with your WiFi credentials and Telegram bot token/chat ID (see the extern declarations in that file for the required variable names).
2. Flash the firmware.
3. Open the serial monitor -- the system will connect to WiFi, sync NTP time, and start the Telegram bot.
4. Send `/start` to the bot to receive the control menu.
5. Calibrate the 4 areas using `CALIB 1`-`CALIB 4` commands -- move the laser to each corner and press SAVE_LIMIT.

---

## Telegram Commands

| Command | Description |
|---------|-------------|
| `/start` | Start monitoring and show the control menu |
| `/stop` | Stop monitoring |
| `/foto` | Capture and send a photo |
| `/simular` | Simulate a pigeon detection (triggers full attack sequence) |
| `/calib 1`-`/calib 4` | Enter calibration mode for area 1-4 |
| `/ver 1`-`/ver 4` | Preview the calibrated area boundary |
| `/claro` | Switch to "daylight" WiFi network |
| `/familia` | Switch to "family" WiFi network |
| `/horarios` | View/edit operating schedules |
| `/config` | View system configuration |
| `/status` | Show current system status |
| `/help` | Show help message |
| `/horario` | Set a schedule time window |
| `/mins` | Set monitoring interval in minutes |
| `/speed` | Set servo speed |

---

## Dataset Collection

For collecting training images (separate from the main firmware):

1. Open `esp32s3_captura_automatica.ino` in Arduino IDE.
2. Select board: ESP32S3 Dev Module, 16MB Flash, OPI PSRAM.
3. Insert a FAT32-formatted microSD card.
4. Upload and monitor at 115200 baud.
5. Process captured images with `Dataset/imagecrop.py`.

See `README_PICS.md` for detailed instructions (in Spanish).

---

## Technical Hardware Information

<!-- Add hardware specifications, wiring diagrams, pin mappings, power requirements, and physical setup details below -->

| Parameter | Value |
|-----------|-------|
| Microcontroller | |
| Camera Module | |
| Servo Motors (Pan) | |
| Servo Motors (Tilt) | |
| Laser Diode | |
| Power Supply | |
| Operating Voltage | |
| WiFi Standard | |
| Flash Size | |
| PSRAM | |
| PCB / Board | |
| Enclosure / Mount | |

---

## License

<!-- Add license information here -->
