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
enum class DeviceState : uint8_t { IDLE, SEND_ATTEMPT, RETRY_WAIT, SUCCESS_HOLD, CANCELLED };

struct SendContext {
  char text[120]; // Changed from const char* to buffer to allow appending coords
  bool urgent;
  bool firstAttempt;
};

static auto deviceState = DeviceState::IDLE;
static SendContext sendCtx = {};
static unsigned long successUntil = 0;
static unsigned long sendStartMs = 0;
static int lastSendErr = ISBD_SUCCESS;
static int lastMoStatus = -1;
static bool lastSawSBDIX = false;
static volatile bool gCancelRequested = false;

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

static void lowPowerDelayMs(uint32_t ms);
static bool netAvailHigh();
static bool waitForNetAvailAndMinCSQ(unsigned long maxWaitMs, int minCSQ, int stableSamples);
static bool waitWithSignalLogsUntilReady(unsigned long totalMs, int minCsq, int stableSamples);
static bool bothButtonsPressed();
static bool cancelRequested();
static void cancelCurrentOperation();
static void applyModemSettings();
static bool ensureModemAwake();
static void sleepModemBestEffort();

// ---------- Low-power delay ----------
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

static bool netAvailHigh() {
#if (PIN_ISBD_NA >= 0)
  return digitalRead(PIN_ISBD_NA) == HIGH;
#else
  return true;
#endif
}

static bool waitForNetAvailAndMinCSQ(const unsigned long maxWaitMs, const int minCSQ, int stableSamples) {
  const unsigned long start = millis();
  unsigned long nextSampleAt = start;
  int goodCount = 0;
  // ReSharper disable once CppDFAConstantConditions
  // ReSharper disable once CppDFAUnreachableCode
  if (stableSamples < 1) stableSamples = 1;

  while (millis() - start < maxWaitMs) {
    if (cancelRequested()) return false;
    updateWaitingBlink();

    if (const unsigned long now = millis(); now < nextSampleAt) {
      lowPowerDelayMs(50);
      continue;
    }

    const bool na = netAvailHigh();
    int csq = -1;
    int csqErr = ISBD_PROTOCOL_ERROR;

    if (na && !modem.isAsleep()) {
      csqErr = modem.getSignalQuality(csq);
      if (csqErr == ISBD_SUCCESS && csq >= minCSQ) {
        goodCount++;
      } else {
        goodCount = 0;
      }
    } else {
      goodCount = 0;
    }

#if !IF_QUIET
    SerialMon.print("NA: ");
    SerialMon.print(na ? "1" : "0");
    SerialMon.print("  CSQ: ");
    if (!na) {
      SerialMon.println("(skip; NA=0)");
    } else if (csqErr == ISBD_SUCCESS) {
      SerialMon.print(csq);
      SerialMon.println("/5");
    } else {
      SerialMon.print("read failed err=");
      SerialMon.println(csqErr);
    }
#endif

    if (goodCount >= stableSamples) return true;
    nextSampleAt += CFG_NA_SAMPLE_MS;
  }

  return false;
}

// ReSharper disable twice CppDFAConstantParameter
static bool waitWithSignalLogsUntilReady(const unsigned long totalMs, const int minCsq, int stableSamples) {
  static constexpr unsigned long kSignalSampleMs = CFG_NA_SAMPLE_MS;
  const unsigned long start = millis();
  const unsigned long end = start + totalMs;
  unsigned long nextSampleAt = start;
  int goodCount = 0;
  // ReSharper disable once CppDFAConstantConditions
  // ReSharper disable once CppDFAUnreachableCode
  if (stableSamples < 1) stableSamples = 1;

  while (true) {
    if (cancelRequested()) return false;
    const unsigned long now = millis();
    if (now >= end) break;
    updateWaitingBlink();

    if (now >= nextSampleAt) {
      const bool na = netAvailHigh();
      int csq = -1;
      int csqErr = ISBD_PROTOCOL_ERROR;

      if (na && !modem.isAsleep()) {
        csqErr = modem.getSignalQuality(csq);
        if (csqErr == ISBD_SUCCESS && csq >= minCsq) {
          goodCount++;
        } else {
          goodCount = 0;
        }
      } else {
        goodCount = 0;
      }

#if !IF_QUIET
      SerialMon.print("NA: ");
      SerialMon.print(na ? "1" : "0");
      SerialMon.print("  CSQ: ");
      if (!na) {
        SerialMon.println("(skip; NA=0)");
      } else if (csqErr == ISBD_SUCCESS) {
        SerialMon.print(csq);
        SerialMon.println("/5");
      } else {
        SerialMon.print("read failed err=");
        SerialMon.println(csqErr);
      }
#endif

      if (goodCount >= stableSamples) return true;
      nextSampleAt += kSignalSampleMs;
      continue;
    }

    unsigned long nextEvent = end;
    if (getPixelMode() == MODE_WAITING) {
      if (const unsigned long nextBlinkAt = getLastBlinkToggle() + kWaitBlinkMs; nextBlinkAt < nextEvent) nextEvent = nextBlinkAt;
    }
    if (nextSampleAt < nextEvent) nextEvent = nextSampleAt;
    if (nextEvent > now) lowPowerDelayMs(nextEvent - now);
  }

  return false;
}

// Library callback (called repeatedly during modem work)
// Blink yellow while waiting.
bool ISBDCallback() {
  if (cancelRequested()) return false;
  updateWaitingBlink();
  return true; // never cancel
}

static void waitForSerialIfEnabled() {
  if constexpr (WAIT_FOR_USB_SERIAL_MS == 0) return;
  const unsigned long start = millis();
  while (!SerialMon && (millis() - start < WAIT_FOR_USB_SERIAL_MS)) { delay(10); }
}

static bool bothButtonsPressed() {
  return digitalRead(BTN_ALERT) == LOW && digitalRead(BTN_SOS) == LOW;
}

static bool cancelRequested() {
  if (deviceState != DeviceState::IDLE && bothButtonsPressed()) gCancelRequested = true;
  return gCancelRequested;
}

static void cancelCurrentOperation() {
  lastSendErr = ISBD_CANCELLED;
  lastMoStatus = -1;
  lastSawSBDIX = false;
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

// ---------- Modem helpers ----------
static void applyModemSettings() {
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);
  modem.useMSSTMWorkaround(CFG_ENABLE_MSSTM_WORKAROUND == 1);
  modem.adjustATTimeout(CFG_MODEM_AT_TIMEOUT_S);
  modem.adjustSendReceiveTimeout(CFG_MODEM_SENDRECV_TIMEOUT_S);
  modem.adjustStartupTimeout(CFG_MODEM_STARTUP_TIMEOUT_S);
  modem.adjustSBDSessionTimeout(CFG_MODEM_SESSION_TIMEOUT_S);
}

static bool ensureModemAwake() {
#if (PIN_ISBD_SLEEP < 0) && (ENABLE_MODEM_SLEEP == 1)
  // No sleep pin wired; can't do real sleep/wake cycling.
#endif

  if (!modem.isAsleep()) {
    applyModemSettings();
#if (PIN_ISBD_RI >= 0)
    modem.enableRingAlerts(true);
#endif
    return true;
  }

  const int err = modem.begin();
  if (err == ISBD_SUCCESS) {
    applyModemSettings();
#if (PIN_ISBD_RI >= 0)
    modem.enableRingAlerts(true);
#endif
    return true;
  }

#if !IF_QUIET
  SerialMon.print("modem.begin() failed, err="); SerialMon.println(err);
  if (err == ISBD_NO_MODEM_DETECTED) SerialMon.println("No modem detected.");
#endif
  return false;
}

static void sleepModemBestEffort() {
  if constexpr (ENABLE_MODEM_SLEEP != 1) return;

  // IridiumSBD::sleep() will only work if a sleep pin was provided to the modem constructor.  [oai_citation:3‡GitHub](https://github.com/mikalhart/IridiumSBD/blob/master/src/IridiumSBD.cpp?utm_source=chatgpt.com)
  const int err = modem.sleep();
  (void)err; // best-effort: ignore errors (e.g., ISBD_NO_SLEEP_PIN)
}

// Build small MO payload: [len8][ASCII bytes...], perform send, and drive NeoPixel states.
static bool sendTextWithIndicators(const char *text, const bool urgent, const bool firstAttempt) {
#if !IF_QUIET
  SerialMon.print("Sending \""); SerialMon.print(text); SerialMon.println("\"...");
#endif
  pixelSetMode(MODE_WAITING);
  if (cancelRequested()) {
    lastSendErr = ISBD_CANCELLED;
    return false;
  }

  if (!ensureModemAwake()) {
    lastSendErr = ISBD_PROTOCOL_ERROR;
    return false;
  }

#if (PIN_ISBD_NA >= 0)
  const unsigned long waitBudget = firstAttempt
    ? CFG_FIRST_ATTEMPT_WAIT_MS
    : (urgent ? CFG_NA_WAIT_URGENT_MS : CFG_NA_WAIT_MAX_MS);
  const int minCsq = firstAttempt ? CFG_FIRST_ATTEMPT_MIN_CSQ : CFG_MIN_CSQ_TO_SEND;
  const int stableSamples = firstAttempt ? CFG_FIRST_ATTEMPT_STABLE_SAMPLES : CFG_MIN_CSQ_STABLE_SAMPLES;
  if (!waitForNetAvailAndMinCSQ(waitBudget, minCsq, stableSamples)) {
#if !IF_QUIET
    SerialMon.println("NA/CSQ not good yet; skipping SBD session this round.");
#endif
    lastSendErr = ISBD_NO_NETWORK;
    sleepModemBestEffort();
    return false;
  }
#endif

  size_t len = strlen(text);
  if (len > 110) len = 110;
  uint8_t mo[1 + 110] = {};
  mo[0] = static_cast<uint8_t>(len);
  memcpy(&mo[1], text, len);

  // Reset parsed SBDIX state so we don't read stale values on a timeout.
  gSBDIXSeen = false;
  gMOStatus = gMOMSN = gMTStatus = gMTMSN = gMTLen = gMTQueued = -1;

  int err = ISBD_PROTOCOL_ERROR;

#if (ENABLE_MT_RECEIVE == 1)
  uint8_t mt[270];
  size_t mtLen = sizeof(mt);
  err = modem.sendReceiveSBDBinary(mo, 1 + len, mt, mtLen);
#else
  // Send-only is usually faster/cheaper (no MT retrieval) when you don't need inbound messages.
  err = modem.sendSBDBinary(mo, 1 + len);
#endif
  if (err == ISBD_IS_ASLEEP) {
    if (ensureModemAwake()) {
#if (ENABLE_MT_RECEIVE == 1)
      err = modem.sendReceiveSBDBinary(mo, 1 + len, mt, mtLen);
#else
      err = modem.sendSBDBinary(mo, 1 + len);
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
static bool edgePressed(const bool current, bool &last) {
  const bool pressed = (last == true && current == false); // HIGH->LOW
  last = current;
  return pressed;
}

static unsigned long computeRetryDelayMs(const int err, const int moStatus, const bool sawSBDIX) {
  // Fast-ish retry for transient errors; slower for "no network service".
  if (sawSBDIX && moStatus == 32) return CFG_RETRY_DELAY_NO_NETWORK_MS;
  if (err == ISBD_NO_NETWORK) return CFG_RETRY_DELAY_NO_NETWORK_MS;
  if (err == ISBD_SENDRECEIVE_TIMEOUT) return CFG_RETRY_DELAY_TIMEOUT_MS;
  return kRetryDelayMs;
}

void setup() {
  // Buttons: active-LOW to GND
  pinMode(BTN_ALERT, INPUT_PULLUP);
  pinMode(BTN_SOS,   INPUT_PULLUP);

  // USB Serial
  SerialMon.begin(115200);
  waitForSerialIfEnabled();

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

void loop() {
  processGps();

  const bool curAlert = digitalRead(BTN_ALERT);
  const bool curSOS   = digitalRead(BTN_SOS);
  const bool bothPressed = (curAlert == LOW && curSOS == LOW);

  if (deviceState != DeviceState::IDLE && bothPressed) {
    gCancelRequested = true;
    deviceState = DeviceState::CANCELLED;
  }

  switch (deviceState) {
    case DeviceState::IDLE: {
      if (const unsigned long now = millis(); now - lastBounceMs > 30) {
        if (!bothPressed && edgePressed(curAlert, lastAlert)) {
#if !IF_QUIET
          SerialMon.println("ALERT button pressed.");
#endif
          sendStartMs = millis();

          // Reset context
          memset(&sendCtx, 0, sizeof(sendCtx));
          strcpy(sendCtx.text, "ALERT");
          sendCtx.urgent = false;
          sendCtx.firstAttempt = true;

          gCancelRequested = false;

          // Attempt GPS (this blocks for up to 15s or until fix/cancel)
          // It appends coordinates to sendCtx.text
          attemptGpsFix(sendCtx.text, cancelRequested);

          deviceState = DeviceState::SEND_ATTEMPT;

        } else if (!bothPressed && edgePressed(curSOS, lastSOS)) {
#if !IF_QUIET
          SerialMon.println("SOS button pressed.");
#endif
          sendStartMs = millis();

          memset(&sendCtx, 0, sizeof(sendCtx));
          strcpy(sendCtx.text, "SOS");
          sendCtx.urgent = true;
          sendCtx.firstAttempt = true;

          gCancelRequested = false;

          attemptGpsFix(sendCtx.text, cancelRequested);

          deviceState = DeviceState::SEND_ATTEMPT;
        }
        lastBounceMs = now;
      }

      if (deviceState == DeviceState::IDLE) {
        lowPowerDelayMs(CFG_IDLE_POLL_MS);
      }
      break;
    }
    case DeviceState::SEND_ATTEMPT: {
      if (cancelRequested()) {
        deviceState = DeviceState::CANCELLED;
        break;
      }
      const bool ok = sendTextWithIndicators(sendCtx.text, sendCtx.urgent, sendCtx.firstAttempt);
      sendCtx.firstAttempt = false;
      deviceState = ok ? DeviceState::SUCCESS_HOLD : DeviceState::RETRY_WAIT;
      break;
    }
    case DeviceState::RETRY_WAIT: {
      if (cancelRequested()) {
        deviceState = DeviceState::CANCELLED;
        break;
      }
      const unsigned long delayMs = computeRetryDelayMs(lastSendErr, lastMoStatus, lastSawSBDIX);
#if !IF_QUIET
      SerialMon.println("Retrying after delay...\n\n");
#endif
      pixelSetMode(MODE_WAITING);
      (void)waitWithSignalLogsUntilReady(delayMs, CFG_MIN_CSQ_TO_SEND, CFG_MIN_CSQ_STABLE_SAMPLES);
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
      deviceState = DeviceState::IDLE;
      break;
    }
  }
}
