# Gemini Project Context: Iridium Satellite Communicator

This project is an Arduino-based satellite communicator using the RockBLOCK 9603 modem and an RP2040-based microcontroller (specifically the Adafruit KB2040). It allows sending "ALERT" or "SOS" messages with GPS coordinates over the Iridium satellite network.

## Project Overview

- **Hardware**: Adafruit KB2040 (RP2040), RockBLOCK 9603 (Iridium SBD modem), u-blox GNSS module, NeoPixel (onboard), and two buttons.
- **Core Functionality**:
    - Trigger "ALERT" or "SOS" messages via physical buttons.
    - Fetch GPS coordinates before sending.
    - Provide visual feedback via NeoPixel colors and patterns.
    - Robust retry logic and power management for the satellite modem.
- **Key Technologies**: PlatformIO, Arduino framework, IridiumSBD library, SparkFun u-blox GNSS library.

## File Structure

- `src/main.cpp`: Main application logic, state machine, and hardware interfacing.
- `include/config.h`: Central configuration for pins, timeouts, logging levels, and power settings.
- `include/print_functions.h`: Diagnostic helpers for parsing and printing modem responses.
- `platformio.ini`: PlatformIO configuration, including dependencies and RP2040-specific settings.
- `archive/`: Contains older versions and experiments (`.ino` files).
- `Eagle Files/`: PCB design files.

## Building and Running

This project uses **PlatformIO**.

- **Build**: `pio run`
- **Upload**: `pio run --target upload`
- **Serial Monitor**: `pio device monitor` (Default baud rate: 115200)

## Development Conventions

- **Configuration**: All hardware-specific and behavioral settings should be modified in `include/config.h`. Avoid hardcoding values in `main.cpp`.
- **State Machine**: The application logic in `loop()` is driven by a `DeviceState` enum. Transitions should be handled carefully to maintain correct NeoPixel and modem status.
- **Logging**: Use the `LOG_LEVEL` macro in `config.h` to control verbosity.
    - `0` (QUIET): Minimal output.
    - `1` (COMPACT): Structured one-liners for modem transactions.
    - `2` (VERBOSE): Full raw diagnostics.
- **Modem Interaction**: Use the `IridiumSBD` library. Note the custom console and diagnostic callbacks for enhanced logging.
- **Power Management**: The project includes logic for underclocking the RP2040 and sleep/wake cycling for the RockBLOCK modem (if `PIN_ISBD_SLEEP` is configured).

## Key Constants & Modes

- **Buttons**:
    - `BTN_ALERT` (D9): Sends "ALERT" + GPS.
    - `BTN_SOS` (D8): Sends "SOS" + GPS.
- **NeoPixel States**:
    - `OFF`: IDLE
    - `BLINK YELLOW`: WAITING (Modem/Network activity)
    - `SOLID BLUE`: GPS_SEARCH
    - `SOLID GREEN`: SUCCESS (Holds for 10s)
    - `SOLID RED`: FAIL (During retry delay)
- **Cancellation**: Pressing both buttons simultaneously during an active operation will trigger a cancellation.
