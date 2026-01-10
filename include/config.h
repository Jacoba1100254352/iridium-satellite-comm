#ifndef IRIDIUM_SATELLITE_COMM_CONFIG_H
#define IRIDIUM_SATELLITE_COMM_CONFIG_H

#ifndef SerialMon
  #define SerialMon Serial
#endif

// ===== Verbosity / logging level =====
// 0 = QUIET
// 1 = COMPACT
// 2 = VERBOSE
#ifndef LOG_LEVEL
  #define LOG_LEVEL 1
#endif

// Backwards-compat knobs (optional)
#ifndef DIAGNOSTICS
  #define DIAGNOSTICS true
#endif

// Helpers
#if (LOG_LEVEL == 1)
  #define IF_COMPACT 1
  #define IF_VERBOSE 0
  #define IF_QUIET 0
#elif (LOG_LEVEL == 2)
  #define IF_VERBOSE 1
  #define IF_COMPACT 0
  #define IF_QUIET 0
#else
  #define IF_QUIET 1
  #define IF_VERBOSE 0
  #define IF_COMPACT 0
#endif

// ===== Power / energy knobs =====

// Wait for USB Serial at boot (set to 0 for fastest boot)
#ifndef WAIT_FOR_USB_SERIAL_MS
  #define WAIT_FOR_USB_SERIAL_MS 0
#endif

// NeoPixel: power-gate the onboard NeoPixel using NEOPIXEL_POWER when available.
// (KB2040 exposes a dedicated NeoPixel power control pin.)  [oai_citation:4‡Adafruit Learning System](https://learn.adafruit.com/adafruit-kb2040/circuitpython-pins-and-modules?utm_source=chatgpt.com)
#ifndef ENABLE_NEOPIXEL_POWER_GATING
  #define ENABLE_NEOPIXEL_POWER_GATING 0
#endif

#ifndef CFG_NEOPIXEL_BRIGHTNESS
  #define CFG_NEOPIXEL_BRIGHTNESS 50
#endif

// Modem sleep support (requires SLP pin wiring; see RockBLOCK v3.F caution)  [oai_citation:5‡Invalid URL](data:text/plain;charset=utf-8,Invalid%20citation)
#ifndef ENABLE_MODEM_SLEEP
  #define ENABLE_MODEM_SLEEP 0
#endif

// RockBLOCK control pins (set to actual GPIO numbers, or leave -1 to disable)
#ifndef PIN_ISBD_SLEEP
  #define PIN_ISBD_SLEEP -1
#endif

#ifndef PIN_ISBD_RI
  #define PIN_ISBD_RI -1
#endif

// If you do not need inbound (MT) messages, leave disabled for faster/cheaper sessions.
#ifndef ENABLE_MT_RECEIVE
  #define ENABLE_MT_RECEIVE 1
#endif

// Timing
#ifndef CFG_RETRY_DELAY_MS
  #define CFG_RETRY_DELAY_MS 10000UL
#endif
#ifndef CFG_RETRY_DELAY_NO_NETWORK_MS
  #define CFG_RETRY_DELAY_NO_NETWORK_MS 20000UL
#endif
#ifndef CFG_RETRY_DELAY_TIMEOUT_MS
  #define CFG_RETRY_DELAY_TIMEOUT_MS 10000UL
#endif
#ifndef CFG_SUCCESS_HOLD_MS
  #define CFG_SUCCESS_HOLD_MS 10000UL
#endif
#ifndef CFG_WAIT_BLINK_MS
  #define CFG_WAIT_BLINK_MS 250UL
#endif

// Idle loop pacing (prevents “spin hot” while doing nothing)
#ifndef CFG_IDLE_POLL_MS
  #define CFG_IDLE_POLL_MS 10UL
#endif

// Modem timeouts (seconds): tuned for reliability and quick success.
#ifndef CFG_MODEM_AT_TIMEOUT_S
  #define CFG_MODEM_AT_TIMEOUT_S 30
#endif
#ifndef CFG_MODEM_SENDRECV_TIMEOUT_S
  #define CFG_MODEM_SENDRECV_TIMEOUT_S 300
#endif
#ifndef CFG_MODEM_STARTUP_TIMEOUT_S
  #define CFG_MODEM_STARTUP_TIMEOUT_S 120
#endif
#ifndef CFG_MODEM_SESSION_TIMEOUT_S
  #define CFG_MODEM_SESSION_TIMEOUT_S 420
#endif

#endif // IRIDIUM_SATELLITE_COMM_CONFIG_H
