# KB2040 RockBLOCK Iridium Beacon

This repository contains the current KB2040 + RockBLOCK 9603 beacon firmware, along with a simplified serial-controlled base-case firmware for direct modem testing.

## Main Firmware

- Main firmware entry point: `src/main.cpp`
- Target board: Adafruit KB2040
- Primary PlatformIO environment: `adafruit_kb2040`

## Upload The Main Firmware

Build and upload `src/main.cpp` to the KB2040:

```bash
pio run -e adafruit_kb2040 -t upload
```

Open the serial monitor for the main firmware:

```bash
pio device monitor -e adafruit_kb2040
```

If PlatformIO does not auto-detect the board, put the KB2040 into BOOTSEL mode and specify the upload port explicitly:

```bash
pio run -e adafruit_kb2040 -t upload --upload-port /dev/cu.usbmodemXXXX
```

## Upload The Base-Case Firmware

Build and upload the serial-command base case:

```bash
pio run -e adafruit_kb2040_basecase -t upload
```

Open the serial monitor for the base-case firmware:

```bash
pio device monitor -e adafruit_kb2040_basecase
```

## Main Send Flow

```mermaid
flowchart TD
  A["Boot"] --> B["setup(): Serial1.begin, sleep pin HIGH, 4s RockBLOCK prime"]
  B --> C["IDLE: background GNSS polling only"]
  C --> D["Button press"]
  D --> E["attemptGpsFix(): fresh fix if already present; else LKG; else NOFIX"]
  E --> F["SEND_ATTEMPT #n"]
  F --> G{"ensureModemAwake()"}
  G -- "already configured and awake" --> H["Build payload + elapsed-seconds suffix"]
  G -- "not configured / recovery" --> I["modem.begin() -> internalBegin()"]
  I --> I1["power(true)"]
  I1 --> I2["500ms startup settle"]
  I2 --> I3["repeat AT until OK\nup to 120s total startup window"]
  I3 --> I4["ATE1 / AT&D0 / AT&K0 / AT+SBDMTA / AT+CGMR / AT+SBDST=90"]
  I4 --> H

  H --> J["sendSBDBinary()"]
  J --> J1["AT+SBDWB=len"]
  J1 --> J2["wait READY up to 30s"]
  J2 --> J3["write payload + checksum"]
  J3 --> J4["wait checksum/OK up to 30s"]

  J4 --> K{"internal send loop\nup to 90s total"}
  K --> L["AT+SBDIX"]
  L --> M["wait response up to 30s"]
  M --> N{"MO status"}
  N -- "0..4" --> O["SUCCESS"]
  N -- "12 / 14 / 16" --> P["fatal return"]
  N -- "other retryable" --> Q["internal retry wait 10s"]
  Q --> K

  O --> R["SUCCESS_HOLD 10s green"]
  P --> S["outer RETRY_WAIT"]
  K -- "90s total elapsed" --> S

  S --> T["orange blink, log NA/CSQ every 1s"]
  T --> U["outer backoff delay expires"]
  U --> F
```

## Notes

- The main firmware appends the elapsed whole-second send timer to the transmitted payload on each outer send attempt.
- GNSS metadata is attached without blocking the first send when no fresh fix is available.
- The base-case environment exists to isolate modem behavior from the full button/GNSS state machine.
