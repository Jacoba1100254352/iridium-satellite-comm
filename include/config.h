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
  #define LOG_LEVEL 2
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
  #define PIN_ISBD_SLEEP 2
#endif

// RockBLOCK Ring Indicator (RI) pin (wire to KB2040 D3).
// Set to -1 to disable RI handling.
#ifndef PIN_ISBD_RI
  #define PIN_ISBD_RI -1
#endif

// RockBLOCK Network Available (NA / NetAv) pin (wire to KB2040 D4).
// Set to -1 to disable NA gating.
#ifndef PIN_ISBD_NA
  #define PIN_ISBD_NA 4
#endif

// If you do not need inbound (MT) messages, leave disabled for faster/cheaper sessions.
#ifndef ENABLE_MT_RECEIVE
  #define ENABLE_MT_RECEIVE 0
#endif

// Timing
#ifndef CFG_RETRY_DELAY_MS
  #define CFG_RETRY_DELAY_MS 10000UL
#endif
#ifndef CFG_RETRY_DELAY_NO_NETWORK_MS
  #define CFG_RETRY_DELAY_NO_NETWORK_MS 60000UL
#endif
#ifndef CFG_RETRY_DELAY_TIMEOUT_MS
  #define CFG_RETRY_DELAY_TIMEOUT_MS 10000UL
#endif
#ifndef CFG_FAILURE_GRACE_MS
  #define CFG_FAILURE_GRACE_MS 30000UL
#endif
#ifndef CFG_NA_WAIT_MAX_MS
  #define CFG_NA_WAIT_MAX_MS 130000UL
#endif
#ifndef CFG_NA_WAIT_URGENT_MS
  #define CFG_NA_WAIT_URGENT_MS 60000UL
#endif
#ifndef CFG_MIN_CSQ_TO_SEND
  #define CFG_MIN_CSQ_TO_SEND 2
#endif
#ifndef CFG_MIN_CSQ_STABLE_SAMPLES
  #define CFG_MIN_CSQ_STABLE_SAMPLES 3
#endif
#ifndef CFG_FIRST_ATTEMPT_WAIT_MS
  #define CFG_FIRST_ATTEMPT_WAIT_MS 10000UL
#endif
#ifndef CFG_FIRST_ATTEMPT_MIN_CSQ
  #define CFG_FIRST_ATTEMPT_MIN_CSQ 1
#endif
#ifndef CFG_FIRST_ATTEMPT_STABLE_SAMPLES
  #define CFG_FIRST_ATTEMPT_STABLE_SAMPLES 2
#endif
#ifndef CFG_ENABLE_MSSTM_WORKAROUND
  #define CFG_ENABLE_MSSTM_WORKAROUND 1
#endif
#ifndef CFG_NA_SAMPLE_MS
  #define CFG_NA_SAMPLE_MS 2000UL
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
  #define CFG_MODEM_SESSION_TIMEOUT_S 300
#endif

#endif // IRIDIUM_SATELLITE_COMM_CONFIG_H
