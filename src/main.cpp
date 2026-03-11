#include <Arduino.h>
#include <IridiumSBD.h>

#if defined(ARDUINO_ARCH_RP2040)
// Arduino-Pico bundles the Pico SDK, so we can use sleep_ms() for low-power waits.  [oai_citation:0‡Arduino-Pico](https://arduino-pico.readthedocs.io/en/latest/sdk.html)
  #include "pico/stdlib.h"
#endif

#include "../include/config.h"
#include "../include/gps_helper.h"
#include "../include/led_helper.h"
#include "../include/print_functions.h"

// =========================
// Buttons (active-LOW to GND)
// =========================
static constexpr int BTN_ALERT = PIN_BTN_ALERT;
static constexpr int BTN_SOS   = PIN_BTN_SOS;

// =========================
// Timing (override in config.h via CFG_* macros)
// =========================
static constexpr unsigned long kRetryDelayMs    = CFG_RETRY_DELAY_MS;
static constexpr unsigned long kSuccessHoldMs   = CFG_SUCCESS_HOLD_MS;
static constexpr unsigned long kWaitBlinkMs     = CFG_WAIT_BLINK_MS;

// =========================
// State / Types
// =========================
enum class DeviceState : uint8_t { IDLE, MODEM_WAKE_WAIT, SEND_ATTEMPT, RETRY_WAIT, SUCCESS_HOLD, CANCELLED, MODEM_RESET_PULSE };
enum class TwoButtonAction : uint8_t { NONE, SOFT_CANCEL, RESET_PULSE, LATCH_SLEEP };

struct SendContext {
  char text[120]; // Changed from const char* to buffer to allow appending coords
  bool urgent;
  bool firstAttempt;
  uint8_t attemptCount;
};

static auto deviceState = DeviceState::IDLE;
static SendContext sendCtx = {};
static unsigned long successUntil = 0;
static unsigned long sendStartMs = 0;
static int lastSendErr = ISBD_SUCCESS;
static int lastMoStatus = -1;
static bool lastSawSBDIX = false;
static volatile bool gCancelRequested = false;
static bool gModemConfigured = false;
static volatile TwoButtonAction gPendingTwoButtonAction = TwoButtonAction::NONE;
static bool gTwoButtonHoldActive = false;
static bool gTwoButtonLongActionTriggered = false;
static unsigned long gTwoButtonHoldStartMs = 0;
static bool gModemSleepLatched = false;
static unsigned long gStateDeadlineMs = 0;

// SBDIX status (updated by console callback)
// static volatile bool gLastMOSuccess = false;

// Debounce
static bool lastAlert = true; // pullup idle HIGH
static bool lastSOS   = true;
static unsigned long lastBounceMs = 0;

// =========================
// RockBLOCK on Serial1 (UART0: TX=D0, RX=D1)
// =========================
// POWER NOTE (RockBLOCK v3.F+): the SLP pin requires special attention when interfacing
// with 3.3V logic. See Adafruit's RockBLOCK guide.  [oai_citation:1‡Invalid URL](data:text/plain;charset=utf-8,Invalid%20citation)
//
// If you wire the SLP pin, define PIN_ISBD_SLEEP in config.h and the modem can be put
// into low power sleep and woken on-demand.
#if (PIN_ISBD_SLEEP >= 0) && (PIN_ISBD_RI >= 0)
IridiumSBD modem(Serial1, PIN_ISBD_SLEEP, PIN_ISBD_RI);
#elif (PIN_ISBD_SLEEP >= 0)
IridiumSBD modem(Serial1, PIN_ISBD_SLEEP); // NOLINT(cppcoreguidelines-interfaces-global-init)
#else
IridiumSBD modem(Serial1);
#endif

#if DIAGNOSTICS
/**
 * Mirrors raw modem console output and parses completed SBDIX lines into the
 * shared diagnostic fields used by the retry logic and operator-facing logs.
 */
void ISBDConsoleCallback(IridiumSBD *d, const char c) {
#if IF_VERBOSE
  SerialMon.write(c);
#endif

  static char line[128];
  static uint8_t idx = 0;

  if (c == '\r') return;
  if (c == '\n') {
    line[idx] = '\0';
    idx = 0;

    if (line[0] != '\0') {
#if IF_COMPACT
      if (strncmp(line, "+SBDIX:", 7) == 0) {
        SerialMon.print("<< ");
        SerialMon.println(line);
      }
#endif

      // Pretty-print (COMPACT/VERBOSE)
#if IF_COMPACT
      diagIngestConsoleLine(line);
#endif

      // Parse +SBDIX and emit compact/verbose status
      if (strncmp(line, "+SBDIX:", 7) == 0) {
        char *p = line + 7;
        char *endp;
        int vals[6];
        bool success = true;
        for (int & val : vals) {
          val = static_cast<int>(strtol(p, &endp, 10));
          if (p == endp) {
            success = false;
            break;
          }
          p = endp;
          while (*p == ',' || *p == ' ') {
            p++;
          }
        }

        if (success) {
          gMOStatus = vals[0];
          gMOMSN = vals[1];
          gMTStatus = vals[2];
          gMTMSN = vals[3];
          gMTLen = vals[4];
          gMTQueued = vals[5];
          gSBDIXSeen = true;

#if IF_COMPACT
          printSBDIXCompact();
#elif IF_VERBOSE
          printSBDIXLegendOnce();
          printSBDIXVerbose();
#endif
        }
      }
      // gLastMOSuccess = (gMOStatus == 0);
    }
    return;
  }

  if (idx < sizeof(line) - 1) line[idx++] = c; else idx = 0; // guard overflow
}

/**
 * Mirrors library diagnostic output without affecting modem behavior so the
 * serial console can show timing and state transitions during a send.
 */
void ISBDDiagsCallback(IridiumSBD *d, const char c) {
#if IF_VERBOSE
  SerialMon.write(c); // raw only in verbose
#endif

  static char line[128];
  static uint8_t idx = 0;

  if (c == '\r') return;
  if (c == '\n') {
    line[idx] = '\0';
    idx = 0;
#if IF_VERBOSE
    if (line[0] != '\0') { SerialMon.print("DBG: "); SerialMon.println(line); }
#endif
    return;
  }

  if (idx < sizeof(line) - 1) line[idx++] = c; else idx = 0;
}
#endif

// Forward decls
static bool sendTextWithIndicators(const char *text, bool urgent, bool firstAttempt);
static size_t buildOutboundMessage(const char *baseText, char *out, size_t outSize);

static void lowPowerDelayMs(uint32_t ms);
static void waitWithSignalLogs(unsigned long totalMs);
static bool netAvailHigh();
static bool bothButtonsPressed();
static void updateTwoButtonControl();
static bool cancelRequested();
static void cancelCurrentOperation();
static void applyModemSettings();
static bool ensureModemAwake();
static void sleepModemBestEffort();
static unsigned long randomRetryDelayMs(unsigned long maxDelayMs);
static void setModemSleepPin(bool awake);
static void activateLatchedSleepMode();
static void beginModemResetPulse();
static void finishModemResetPulse();
static void beginWakeBeforeSend();
static void startSendRequest(const char *text, bool urgent);

/**
 * Builds the plain-text outbound payload by appending the elapsed whole-second
 * send timer suffix to the current message body for the next outer attempt.
 */
static size_t buildOutboundMessage(const char *baseText, char *out, const size_t outSize) {
  if (outSize == 0) return 0;

  unsigned long elapsedSeconds = 0;
  if (sendStartMs != 0) {
    elapsedSeconds = (millis() - sendStartMs) / 1000UL;
  }

  char suffix[16] = {};
  snprintf(suffix, sizeof(suffix), "%lus", elapsedSeconds);

  const size_t suffixLen = strnlen(suffix, sizeof(suffix));
  const size_t maxBaseLen = (outSize - 1 > suffixLen) ? (outSize - 1 - suffixLen) : 0;
  const size_t baseLen = strnlen(baseText, maxBaseLen);

  memcpy(out, baseText, baseLen);
  memcpy(out + baseLen, suffix, suffixLen);
  out[baseLen + suffixLen] = '\0';
  return baseLen + suffixLen;
}

// ---------- Low-power delay ----------
/**
 * Sleeps in chunks so the MCU can idle without busy-spinning while still
 * waking often enough to service LED updates and cancel checks.
 */
static void lowPowerDelayMs(uint32_t ms) {
#if defined(ARDUINO_ARCH_RP2040)
  // sleep_ms() is Pico SDK; it idles efficiently vs tight spinning.  [oai_citation:2‡Arduino-Pico](https://arduino-pico.readthedocs.io/en/latest/sdk.html)
  while (ms) {
    const uint32_t chunk = (ms > 1000) ? 1000 : ms;
    sleep_ms(chunk);
    ms -= chunk;
  }
#else
  while (ms) {
    const uint32_t chunk = (ms > 1000) ? 1000 : ms;
    delay(chunk);
    ms -= chunk;
  }
#endif
}

/**
 * Runs the outer retry delay while continuing LED animation, button-driven
 * cancel handling, and periodic signal logging for visibility into wait state.
 */
static void waitWithSignalLogs(const unsigned long totalMs) {
  static constexpr unsigned long kSignalSampleMs = CFG_NA_SAMPLE_MS;
  const unsigned long start = millis();
  const unsigned long end = start + totalMs;
  unsigned long nextSampleAt = start;

  while (true) {
    if (cancelRequested()) return;
    const unsigned long now = millis();
    if (now >= end) break;
    updateWaitingBlink();

    if (now >= nextSampleAt) {
#if !IF_QUIET
      if (PIN_ISBD_NA >= 0) {
        SerialMon.print("NA: ");
        SerialMon.println(netAvailHigh() ? "1" : "0");
      }
      if (!modem.isAsleep()) {
        int csq = -1;
        const int err = modem.getSignalQuality(csq);
        if (err == ISBD_SUCCESS) {
          SerialMon.print("CSQ: ");
          SerialMon.print(csq);
          SerialMon.println("/5");
        } else {
          SerialMon.print("CSQ read failed, err=");
          SerialMon.println(err);
        }
      }
#endif
      nextSampleAt += kSignalSampleMs;
      continue;
    }

    unsigned long nextEvent = end;
    if (const PixelMode pixelMode = getPixelMode(); pixelMode == MODE_WAITING || pixelMode == MODE_RETRY_WAIT) {
      const unsigned long blinkMs = (pixelMode == MODE_RETRY_WAIT) ? CFG_RETRY_WAIT_BLINK_MS : kWaitBlinkMs;
      if (const unsigned long nextBlinkAt = getLastBlinkToggle() + blinkMs; nextBlinkAt < nextEvent) {
        nextEvent = nextBlinkAt;
      }
    }
    if (nextSampleAt < nextEvent) nextEvent = nextSampleAt;
    if (nextEvent > now) lowPowerDelayMs(nextEvent - now);
  }
}

/**
 * Returns the current level of the RockBLOCK NetAv pin when present, which is
 * useful for logging but not used as a blocking precondition for sends.
 */
static bool netAvailHigh() {
#if (PIN_ISBD_NA >= 0)
  return digitalRead(PIN_ISBD_NA) == HIGH;
#else
  return true;
#endif
}

// Library callback (called repeatedly during modem work)
// Blink yellow while waiting.
/**
 * Gives the Iridium library a fast cancel hook and keeps the active-send LED
 * animation alive during long blocking modem transactions.
 */
bool ISBDCallback() {
  if (cancelRequested()) return false;
  updateWaitingBlink();
  return true; // never cancel
}

/**
 * Waits briefly for the USB serial monitor at boot when that behavior is
 * enabled in config, otherwise returns immediately for fastest startup.
 */
static void waitForSerialIfEnabled() {
  if constexpr (WAIT_FOR_USB_SERIAL_MS == 0) return;
  const unsigned long start = millis();
  while (!SerialMon && (millis() - start < WAIT_FOR_USB_SERIAL_MS)) { delay(10); }
}

/**
 * Reports whether both user buttons are currently asserted together, which is
 * the gesture used for staged cancel, reset-pulse, and sleep-until-send modes.
 */
static bool bothButtonsPressed() {
  return digitalRead(BTN_ALERT) == LOW && digitalRead(BTN_SOS) == LOW;
}

/**
 * Interprets the dual-button hold gesture when advanced handling is enabled.
 *
 * Behavior:
 * - release before CFG_TWO_BUTTON_SOFT_CANCEL_MAX_MS: soft-cancel the active operation
 * - release after that but before CFG_TWO_BUTTON_SLEEP_LATCH_TRIGGER_MS: pulse the
 *   RockBLOCK sleep pin low for CFG_TWO_BUTTON_SLEEP_PULSE_MS
 * - hold through CFG_TWO_BUTTON_SLEEP_LATCH_TRIGGER_MS: immediately latch the
 *   RockBLOCK asleep until the next send request wakes it
 *
 * The first two actions only trigger on release. The long-hold action triggers
 * as soon as the threshold is reached so the user does not need to release to
 * force the modem into the latched-sleep state.
 */
static void updateTwoButtonControl() {
#if (ENABLE_ADVANCED_TWO_BUTTON_ACTIONS == 1)
  const bool bothPressed = bothButtonsPressed();
  const unsigned long now = millis();

  if (bothPressed) {
    if (!gTwoButtonHoldActive) {
      gTwoButtonHoldActive = true;
      gTwoButtonLongActionTriggered = false;
      gTwoButtonHoldStartMs = now;
      return;
    }

    if (!gTwoButtonLongActionTriggered &&
        (now - gTwoButtonHoldStartMs) >= CFG_TWO_BUTTON_SLEEP_LATCH_TRIGGER_MS) {
      gTwoButtonLongActionTriggered = true;
      activateLatchedSleepMode();
      if (deviceState != DeviceState::IDLE) {
        gPendingTwoButtonAction = TwoButtonAction::LATCH_SLEEP;
        gCancelRequested = true;
      }
    }
    return;
  }

  if (!gTwoButtonHoldActive) return;

  const unsigned long heldMs = now - gTwoButtonHoldStartMs;
  const bool longActionTriggered = gTwoButtonLongActionTriggered;
  gTwoButtonHoldActive = false;
  gTwoButtonLongActionTriggered = false;

  if (longActionTriggered) return;

  if (heldMs < CFG_TWO_BUTTON_SOFT_CANCEL_MAX_MS) {
    if (deviceState != DeviceState::IDLE) {
      gPendingTwoButtonAction = TwoButtonAction::SOFT_CANCEL;
      gCancelRequested = true;
    }
    return;
  }

  if (heldMs < CFG_TWO_BUTTON_SLEEP_LATCH_TRIGGER_MS) {
    if (deviceState == DeviceState::IDLE) {
      beginModemResetPulse();
    } else {
      gPendingTwoButtonAction = TwoButtonAction::RESET_PULSE;
      gCancelRequested = true;
    }
  }
#else
  if (deviceState != DeviceState::IDLE && bothButtonsPressed()) {
    gCancelRequested = true;
  }
#endif
}

/**
 * Centralizes cancel evaluation so every long-running path uses the same
 * button gesture handling before deciding whether to abort current work.
 */
static bool cancelRequested() {
  updateTwoButtonControl();
  return gCancelRequested;
}

/**
 * Performs the common software-side cleanup for a cancelled send flow without
 * rebooting the MCU: clear send timing/state, stop LEDs, and force modem
 * configuration to be rebuilt on the next attempt.
 */
static void cancelCurrentOperation() {
  lastSendErr = ISBD_CANCELLED;
  lastMoStatus = -1;
  lastSawSBDIX = false;
  gModemConfigured = false;
  pixelSetMode(MODE_IDLE);
  successUntil = 0;
  sendStartMs = 0;
  sleepModemBestEffort();
#if !IF_QUIET
  SerialMon.println("Operation cancelled.");
#endif
  while (bothButtonsPressed()) {
    lowPowerDelayMs(25);
  }
  gCancelRequested = false;
}

/**
 * Drives the RockBLOCK sleep-control pin directly when it is wired, allowing
 * the firmware to hold the modem awake or asleep outside library-managed sleep.
 */
static void setModemSleepPin(const bool awake) {
#if (PIN_ISBD_SLEEP >= 0)
  digitalWrite(PIN_ISBD_SLEEP, awake ? HIGH : LOW);
#else
  (void)awake;
#endif
}

/**
 * Forces the RockBLOCK into an idle latched-sleep state and leaves it there
 * until the next send request explicitly wakes it and waits for recharge.
 */
static void activateLatchedSleepMode() {
#if (PIN_ISBD_SLEEP >= 0)
  setModemSleepPin(false);
  gModemSleepLatched = true;
  gModemConfigured = false;
  pixelSetMode(MODE_IDLE);
  successUntil = 0;
#endif
}

/**
 * Starts the medium-hold reset pulse by holding the RockBLOCK sleep pin low
 * for CFG_TWO_BUTTON_SLEEP_PULSE_MS before returning the device to idle.
 */
static void beginModemResetPulse() {
#if (PIN_ISBD_SLEEP >= 0)
  gPendingTwoButtonAction = TwoButtonAction::NONE;
  gModemSleepLatched = false;
  gModemConfigured = false;
  setModemSleepPin(false);
  gStateDeadlineMs = millis() + CFG_TWO_BUTTON_SLEEP_PULSE_MS;
  pixelSetMode(MODE_IDLE);
#if !IF_QUIET
  SerialMon.print("RockBLOCK sleep pulse for ");
  SerialMon.print(CFG_TWO_BUTTON_SLEEP_PULSE_MS);
  SerialMon.println(" ms.");
#endif
  deviceState = DeviceState::MODEM_RESET_PULSE;
#else
  gPendingTwoButtonAction = TwoButtonAction::NONE;
  deviceState = DeviceState::IDLE;
#endif
}

/**
 * Ends the medium-hold reset pulse, reasserts the sleep pin high, and leaves
 * the next send to perform a full modem reconfiguration from idle.
 */
static void finishModemResetPulse() {
#if (PIN_ISBD_SLEEP >= 0)
  setModemSleepPin(true);
#endif
  gStateDeadlineMs = 0;
  gModemConfigured = false;
  pixelSetMode(MODE_IDLE);
  deviceState = DeviceState::IDLE;
}

/**
 * Wakes a previously latched-asleep RockBLOCK, starts the yellow send LED
 * immediately, and delays the first send attempt for the configured recharge
 * interval so the modem's bulk capacitance can refill.
 */
static void beginWakeBeforeSend() {
#if (PIN_ISBD_SLEEP >= 0)
  setModemSleepPin(true);
#endif
  gModemSleepLatched = false;
  gModemConfigured = false;
  gStateDeadlineMs = millis() + CFG_MODEM_WAKE_BEFORE_SEND_MS;
  pixelSetMode(MODE_WAITING);
#if !IF_QUIET
  SerialMon.print("Waking RockBLOCK before send for ");
  SerialMon.print(CFG_MODEM_WAKE_BEFORE_SEND_MS);
  SerialMon.println(" ms.");
#endif
  deviceState = DeviceState::MODEM_WAKE_WAIT;
}

/**
 * Initializes a new button-driven send request, captures GNSS/LKG metadata,
 * clears prior cancel state, and either enters the send state immediately or
 * starts the pre-send wake delay when the modem was latched asleep.
 */
static void startSendRequest(const char *text, const bool urgent) {
  sendStartMs = millis();
  memset(&sendCtx, 0, sizeof(sendCtx));
  strncpy(sendCtx.text, text, sizeof(sendCtx.text) - 1);
  sendCtx.urgent = urgent;
  sendCtx.firstAttempt = true;
  sendCtx.attemptCount = 0;
  gCancelRequested = false;
  gPendingTwoButtonAction = TwoButtonAction::NONE;
  const bool wakingFromLatchedSleep = (ENABLE_ADVANCED_TWO_BUTTON_ACTIONS == 1) && gModemSleepLatched;
  if (wakingFromLatchedSleep) {
    beginWakeBeforeSend();
  }
  attemptGpsFix(sendCtx.text, cancelRequested);

  if (!wakingFromLatchedSleep) deviceState = DeviceState::SEND_ATTEMPT;
}

// ---------- Modem helpers ----------
/**
 * Applies the project's chosen Iridium library settings so each fresh modem
 * session uses the intended power profile, timeout values, and MSSTM policy.
 */
static void applyModemSettings() {
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);
#if (CFG_ENABLE_MSSTM_WORKAROUND >= 0)
  modem.useMSSTMWorkaround(CFG_ENABLE_MSSTM_WORKAROUND == 1);
#endif
  modem.adjustATTimeout(CFG_MODEM_AT_TIMEOUT_S);
  modem.adjustSendReceiveTimeout(CFG_MODEM_SENDRECV_TIMEOUT_S);
  modem.adjustStartupTimeout(CFG_MODEM_STARTUP_TIMEOUT_S);
  modem.adjustSBDSessionTimeout(CFG_MODEM_SESSION_TIMEOUT_S);
}

/**
 * Runs modem.begin() and, on success, applies project-level modem settings so
 * later send attempts can skip redundant begin/configure work while awake.
 */
static bool beginAndConfigureModem() {
  const int err = modem.begin();
  if (err == ISBD_SUCCESS || err == ISBD_ALREADY_AWAKE) {
    applyModemSettings();
#if (PIN_ISBD_RI >= 0)
    modem.enableRingAlerts(true);
#endif
    gModemConfigured = true;
    return true;
  }

#if !IF_QUIET
  SerialMon.print("modem.begin() failed, err="); SerialMon.println(err);
  if (err == ISBD_NO_MODEM_DETECTED) SerialMon.println("No modem detected.");
#endif
  gModemConfigured = false;
  return false;
}

/**
 * Ensures the modem is ready for use, reusing a known-good configured session
 * when possible and falling back to a full begin/configure when required.
 */
static bool ensureModemAwake() {
#if (PIN_ISBD_SLEEP < 0) && (ENABLE_MODEM_SLEEP == 1)
  // No sleep pin wired; can't do real sleep/wake cycling.
#endif

  if (gModemConfigured && !modem.isAsleep()) {
    return true;
  }

  return beginAndConfigureModem();
}

/**
 * Puts the modem to sleep only when library-managed sleep is enabled; in the
 * default fast-send profile this is intentionally a no-op.
 */
static void sleepModemBestEffort() {
  if constexpr (ENABLE_MODEM_SLEEP != 1) return;

  // IridiumSBD::sleep() will only work if a sleep pin was provided to the modem constructor.  [oai_citation:3‡GitHub](https://github.com/mikalhart/IridiumSBD/blob/master/src/IridiumSBD.cpp?utm_source=chatgpt.com)
  const int err = modem.sleep();
  (void)err; // best-effort: ignore errors (e.g., ISBD_NO_SLEEP_PIN)
}

/**
 * Adds a small randomized jitter to retry delays so repeated sends do not
 * re-attempt on the exact same cadence after transient network failures.
 */
static unsigned long randomRetryDelayMs(const unsigned long maxDelayMs) {
  if (maxDelayMs == 0) return 0;
  return static_cast<unsigned long>(random(maxDelayMs + 1UL));
}

// Build small MO payload: [len8][ASCII bytes...], perform send, and drive NeoPixel states.
/**
 * Performs one outer send attempt: build the text payload for this attempt,
 * ensure the modem is ready, run the blocking Iridium send call, and translate
 * the library result into local success/failure state for the outer loop.
 */
static bool sendTextWithIndicators(const char *text, const bool urgent, const bool firstAttempt) {
#if !IF_QUIET
  char outboundText[111] = {};
  const size_t outboundLen = buildOutboundMessage(text, outboundText, sizeof(outboundText));
  SerialMon.print("Sending \""); SerialMon.print(outboundText); SerialMon.println("\"...");
#else
  char outboundText[111] = {};
  const size_t outboundLen = buildOutboundMessage(text, outboundText, sizeof(outboundText));
#endif
  (void)urgent;
  pixelSetMode(MODE_WAITING);
  if (cancelRequested()) {
    lastSendErr = ISBD_CANCELLED;
    return false;
  }

  if (!ensureModemAwake()) {
    lastSendErr = ISBD_PROTOCOL_ERROR;
    return false;
  }

#if !IF_QUIET
  if (firstAttempt) {
    SerialMon.println("First attempt: skipping NA/CSQ pre-gate.");
  }
#endif

  uint8_t mo[110] = {};
  memcpy(mo, outboundText, outboundLen);

  // Reset parsed SBDIX state so we don't read stale values on a timeout.
  gSBDIXSeen = false;
  gMOStatus = gMOMSN = gMTStatus = gMTMSN = gMTLen = gMTQueued = -1;

  int err = ISBD_PROTOCOL_ERROR;

#if (ENABLE_MT_RECEIVE == 1)
  uint8_t mt[270];
  size_t mtLen = sizeof(mt);
  err = modem.sendReceiveSBDBinary(mo, outboundLen, mt, mtLen);
#else
  // Send-only is usually faster/cheaper (no MT retrieval) when you don't need inbound messages.
  err = modem.sendSBDBinary(mo, outboundLen);
#endif
  if (err == ISBD_IS_ASLEEP) {
    gModemConfigured = false;
    if (ensureModemAwake()) {
#if (ENABLE_MT_RECEIVE == 1)
      err = modem.sendReceiveSBDBinary(mo, outboundLen, mt, mtLen);
#else
      err = modem.sendSBDBinary(mo, outboundLen);
#endif
    }
  }

  const bool sawSBDIX = gSBDIXSeen;
  const int  moStatus = gMOStatus;
  gSBDIXSeen = false;
  lastSendErr = err;
  lastMoStatus = moStatus;
  lastSawSBDIX = sawSBDIX;

  const bool moReportedSuccess = sawSBDIX && (moStatus == 0 || moStatus == 1);
  const bool timedOutButSent   = (err == ISBD_SENDRECEIVE_TIMEOUT && moReportedSuccess);

  if (err != ISBD_SUCCESS && !timedOutButSent) {
    if (err == ISBD_PROTOCOL_ERROR || err == ISBD_NO_MODEM_DETECTED) {
      gModemConfigured = false;
    }
#if !IF_QUIET
    SerialMon.print("SBD send failed, err="); SerialMon.print(err); SerialMon.print(".\tReason:");
    switch (err) {
      case ISBD_ALREADY_AWAKE:       SerialMon.println("Already awake."); break;
      case ISBD_SERIAL_FAILURE:      SerialMon.println("Serial failure."); break;
      case ISBD_PROTOCOL_ERROR:      SerialMon.println("Protocol error."); break;
      case ISBD_CANCELLED:           SerialMon.println("Cancelled by callback."); break;
      case ISBD_NO_MODEM_DETECTED:   SerialMon.println("No modem detected."); break;
      case ISBD_SBDIX_FATAL_ERROR:   SerialMon.println("SBDIX fatal error."); break;
      case ISBD_SENDRECEIVE_TIMEOUT: SerialMon.println("Timeout."); break;
      case ISBD_RX_OVERFLOW:         SerialMon.println("Receive overflow."); break;
      case ISBD_REENTRANT:           SerialMon.println("Reentrant call."); break;
      case ISBD_IS_ASLEEP:           SerialMon.println("Modem is asleep."); break;
      case ISBD_NO_SLEEP_PIN:        SerialMon.println("No sleep pin configured."); break;
      case ISBD_NO_NETWORK:          SerialMon.println("No network service."); break;
      case ISBD_MSG_TOO_LONG:        SerialMon.println("Message too long."); break;
      default:                       SerialMon.println("Unknown error."); break;
    }
    if (sawSBDIX) {
      SerialMon.print("MO-status from modem: ");
      SerialMon.print(moStatus);
      SerialMon.print(" (");
      SerialMon.print(moStatusToStr(moStatus));
      SerialMon.println(")");
      if (moStatus == 32) SerialMon.println("Hint: No network service — move to clear sky; try for CSQ >= 2.");
    }
#endif

    sleepModemBestEffort();
    return false;
  }

#if !IF_QUIET
  if (timedOutButSent) SerialMon.println("Send OK (modem reported MO success after timeout).");
  else                 SerialMon.println("Send OK.");
  if (sawSBDIX) {
    SerialMon.print("MO-status="); SerialMon.print(moStatus); SerialMon.print(" ("); SerialMon.print(moStatusToStr(moStatus)); SerialMon.println(")");
  }
#endif

  pixelSetMode(MODE_SUCCESS);
  successUntil = millis() + kSuccessHoldMs;

#if !IF_QUIET
  SerialMon.print("Successfully Transmitted: \"");
  SerialMon.print(text);
  SerialMon.println("\"");

  if (sendStartMs != 0) {
    const unsigned long elapsedMs = millis() - sendStartMs;
    SerialMon.print("Elapsed since button press: ");
    SerialMon.print(elapsedMs / 1000);
    SerialMon.print("s ");
    SerialMon.print(elapsedMs % 1000);
    SerialMon.println("ms");
    sendStartMs = 0;
  }
#endif

  // Immediately sleep the modem after a successful send.
  sleepModemBestEffort();
  return true;
}

// edge detection for active-LOW buttons
/**
 * Detects a single falling-edge press event for an active-low button while
 * updating the stored prior state for the next pass through the loop.
 */
static bool edgePressed(const bool current, bool &last) {
  const bool pressed = (last == true && current == false); // HIGH->LOW
  last = current;
  return pressed;
}

/**
 * Chooses the next outer retry delay from the last library error, MO status,
 * and attempt number so early retries are aggressive and repeated no-network
 * cases back off more deliberately.
 */
static unsigned long computeRetryDelayMs(const int err, const int moStatus, const bool sawSBDIX, const uint8_t attemptCount) {
  if (sawSBDIX && moStatus == 35) return randomRetryDelayMs(kRetryDelayMs);
  if (sawSBDIX && moStatus == 33) return CFG_RETRY_DELAY_ANTENNA_MS;
  if (err == ISBD_PROTOCOL_ERROR || err == ISBD_NO_MODEM_DETECTED) return CFG_RETRY_DELAY_TIMEOUT_MS;

  if (attemptCount <= 2) return randomRetryDelayMs(kRetryDelayMs);
  if (attemptCount <= 4) {
    if (sawSBDIX && moStatus == 32) return randomRetryDelayMs(CFG_RETRY_DELAY_NO_NETWORK_MS);
    if (err == ISBD_NO_NETWORK || err == ISBD_SENDRECEIVE_TIMEOUT) return randomRetryDelayMs(CFG_RETRY_DELAY_NO_NETWORK_MS);
    return randomRetryDelayMs(CFG_RETRY_DELAY_TIMEOUT_MS);
  }

  if (sawSBDIX && moStatus == 32) return CFG_RETRY_DELAY_SLOW_MS;
  if (err == ISBD_NO_NETWORK || err == ISBD_SENDRECEIVE_TIMEOUT) return CFG_RETRY_DELAY_SLOW_MS;
  return CFG_RETRY_DELAY_TIMEOUT_MS;
}

/**
 * Initializes buttons, LEDs, GNSS, UART, and the RockBLOCK control pins, then
 * primes the modem power path before the device enters its idle event loop.
 */
void setup() {
  // Buttons: active-LOW to GND
  pinMode(BTN_ALERT, INPUT_PULLUP);
  pinMode(BTN_SOS,   INPUT_PULLUP);

  // USB Serial
  SerialMon.begin(115200);
  waitForSerialIfEnabled();
  randomSeed(micros());

  setupLeds();

  setupGps();

  Serial1.begin(19200);

  // If sleep pin is wired, activate the modem (wake it up)
#if (PIN_ISBD_SLEEP >= 0)
  pinMode(PIN_ISBD_SLEEP, OUTPUT);
  // FORCE AWAKE immediately
  digitalWrite(PIN_ISBD_SLEEP, HIGH);

  // CRITICAL: Give the capacitors time to charge through your switch!
  // Since your switch is high-resistance, the capacitors charge slower.
  // We give it 4 seconds of "priming" time.
  SerialMon.println("Priming RockBLOCK capacitors...");
  delay(4000);
#endif

#if (PIN_ISBD_NA >= 0)
  pinMode(PIN_ISBD_NA, INPUT);
#endif

#if !IF_QUIET
  SerialMon.println("KB2040 + RockBLOCK + GNSS");
  SerialMon.println("Press D9 (ALERT) or D8 (SOS) to send.");

  SerialMon.println();
  SerialMon.println("SBDIX fields explanation:");
  SerialMon.println("  MO-status:  Mobile Originated status (e.g., 0=success, 32=no network service)");
  SerialMon.println("  MOMSN:      Mobile Originated Message Sequence Number (increments with each send)");
  SerialMon.println("  MT-status:  Mobile Terminated status (0=no message, 1=message received, 2=error)");
  SerialMon.println("  MTMSN:      Mobile Terminated Message Sequence Number (for the received message)");
  SerialMon.println("  MT-length:  Length in bytes of the received Mobile Terminated message");
  SerialMon.println("  MT-queued:  Number of pending Mobile Terminated messages still waiting on the server");
  SerialMon.println();

  printModemGlossaryOnce();
#endif
}

/**
 * Runs the top-level device state machine, including button handling, wake
 * delays, send attempts, retry backoff, success hold, and cancel/reset actions.
 */
void loop() {
  processGps();
  updateTwoButtonControl();

  const bool curAlert = digitalRead(BTN_ALERT);
  const bool curSOS   = digitalRead(BTN_SOS);
  const bool bothPressed = (curAlert == LOW && curSOS == LOW);

  switch (deviceState) {
    case DeviceState::IDLE: {
      if (const unsigned long now = millis(); now - lastBounceMs > 30) {
        if (!bothPressed && edgePressed(curAlert, lastAlert)) {
#if !IF_QUIET
          SerialMon.println("ALERT button pressed.");
#endif
          startSendRequest("ALERT", false);

        } else if (!bothPressed && edgePressed(curSOS, lastSOS)) {
#if !IF_QUIET
          SerialMon.println("SOS button pressed.");
#endif
          startSendRequest("SOS", true);
        }
        lastBounceMs = now;
      }

      if (deviceState == DeviceState::IDLE) {
        lowPowerDelayMs(CFG_IDLE_POLL_MS);
      }
      break;
    }
    case DeviceState::MODEM_WAKE_WAIT: {
      if (cancelRequested()) {
        deviceState = DeviceState::CANCELLED;
        break;
      }
      updateWaitingBlink();
      if (gStateDeadlineMs != 0 && millis() >= gStateDeadlineMs) {
        gStateDeadlineMs = 0;
        deviceState = DeviceState::SEND_ATTEMPT;
      } else {
        lowPowerDelayMs(CFG_IDLE_POLL_MS);
      }
      break;
    }
    case DeviceState::SEND_ATTEMPT: {
      if (cancelRequested()) {
        deviceState = DeviceState::CANCELLED;
        break;
      }
      ++sendCtx.attemptCount;
#if !IF_QUIET
      SerialMon.print("Send attempt #");
      SerialMon.println(sendCtx.attemptCount);
#endif
      const bool ok = sendTextWithIndicators(sendCtx.text, sendCtx.urgent, sendCtx.firstAttempt);
      sendCtx.firstAttempt = false;
      if (ok) {
        deviceState = DeviceState::SUCCESS_HOLD;
      } else if (lastSendErr == ISBD_CANCELLED || gPendingTwoButtonAction != TwoButtonAction::NONE) {
        deviceState = DeviceState::CANCELLED;
      } else {
        deviceState = DeviceState::RETRY_WAIT;
      }
      break;
    }
    case DeviceState::RETRY_WAIT: {
      if (cancelRequested()) {
        deviceState = DeviceState::CANCELLED;
        break;
      }
      const unsigned long delayMs = computeRetryDelayMs(lastSendErr, lastMoStatus, lastSawSBDIX, sendCtx.attemptCount);
#if !IF_QUIET
      SerialMon.print("Retrying after ");
      SerialMon.print(delayMs);
      SerialMon.println(" ms...\n");
#endif
      pixelSetMode(MODE_RETRY_WAIT);
      waitWithSignalLogs(delayMs);
      deviceState = cancelRequested() ? DeviceState::CANCELLED : DeviceState::SEND_ATTEMPT;
      break;
    }
    case DeviceState::SUCCESS_HOLD: {
      if (successUntil != 0 && millis() >= successUntil) {
        pixelSetMode(MODE_IDLE);
        successUntil = 0;
        deviceState = DeviceState::IDLE;
      } else {
        lowPowerDelayMs(CFG_IDLE_POLL_MS);
      }
      break;
    }
    case DeviceState::CANCELLED: {
      cancelCurrentOperation();
      if (gPendingTwoButtonAction == TwoButtonAction::RESET_PULSE) {
        beginModemResetPulse();
      } else {
        gPendingTwoButtonAction = TwoButtonAction::NONE;
        deviceState = DeviceState::IDLE;
      }
      break;
    }
    case DeviceState::MODEM_RESET_PULSE: {
      if (gStateDeadlineMs != 0 && millis() >= gStateDeadlineMs) {
        finishModemResetPulse();
      } else {
        lowPowerDelayMs(CFG_IDLE_POLL_MS);
      }
      break;
    }
  }
}
