#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <IridiumSBD.h>

#if defined(ARDUINO_ARCH_RP2040)
// Arduino-Pico bundles the Pico SDK, so we can use sleep_ms() for low-power waits.  [oai_citation:0‡Arduino-Pico](https://arduino-pico.readthedocs.io/en/latest/sdk.html)
  #include "pico/stdlib.h"
#endif

#include "../include/config.h"
#include "../include/print_functions.h"

// =========================
// Buttons (active-LOW to GND)
// =========================
static constexpr int BTN_ALERT = 9;   // D9 → GND sends "ALERT"
static constexpr int BTN_SOS   = 8;   // D8 → GND sends "SOS"

// =========================
// Timing (override in config.h via CFG_* macros)
// =========================
static constexpr unsigned long kRetryDelayMs    = CFG_RETRY_DELAY_MS;
static constexpr unsigned long kSuccessHoldMs   = CFG_SUCCESS_HOLD_MS;
static constexpr unsigned long kWaitBlinkMs     = CFG_WAIT_BLINK_MS;

// =========================
// NeoPixel (KB2040 onboard)
// =========================
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

#if defined(NEOPIXEL_POWER)
static const int NEOPIXEL_PWR = NEOPIXEL_POWER;
#endif

// Quick color helpers
static uint32_t C_RED()    { return Adafruit_NeoPixel::Color(255, 0,   0  ); }
static uint32_t C_GREEN()  { return Adafruit_NeoPixel::Color(0,   255, 0  ); }
static uint32_t C_YELLOW() { return Adafruit_NeoPixel::Color(255, 200, 0  ); }
static uint32_t C_OFF()    { return Adafruit_NeoPixel::Color(0,   0,   0  ); }

// =========================
// State / Types
// =========================
enum PixelMode : uint8_t { MODE_IDLE, MODE_WAITING, MODE_FAIL, MODE_SUCCESS };
static PixelMode pixelMode = MODE_IDLE;
static bool waitBlinkOn = false;
static unsigned long lastBlinkToggle = 0;
static unsigned long successUntil = 0;
static unsigned long sendStartMs = 0;
static int lastSendErr = ISBD_SUCCESS;
static int lastMoStatus = -1;
static bool lastSawSBDIX = false;

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
IridiumSBD modem(Serial1, PIN_ISBD_SLEEP);
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
        int a, b, c2, d2, e, f;
        if (sscanf(line + 7, " %d , %d , %d , %d , %d , %d", &a, &b, &c2, &d2, &e, &f) == 6) {
          gMOStatus = a; gMOMSN = b; gMTStatus = c2; gMTMSN = d2; gMTLen = e; gMTQueued = f;
          gSBDIXSeen = true;

#if IF_COMPACT
          printSBDIXCompact();
#elif IF_VERBOSE
          printSBDIXLegendOnce(); printSBDIXVerbose();
#endif
        }
      }
      // gLastMOSuccess = (gMOStatus == 0);
    }
    return;
  }

  if (idx < sizeof(line) - 1) line[idx++] = c; else idx = 0; // guard overflow
}

void ISBDDiagsCallback(IridiumSBD *d, char c) {
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
    if (line[0] != '\0') {
      SerialMon.print("DBG: "); SerialMon.println(line);
    }
#endif
    return;
  }

  if (idx < sizeof(line) - 1) line[idx++] = c; else idx = 0;
}
#endif

// Forward decls
static void pixelShowColor(uint32_t c);
static void pixelSetMode(PixelMode mode);
static void updateWaitingBlink();
static bool sendTextWithIndicators(const char *text, bool urgent, bool firstAttempt);

static void lowPowerDelayMs(uint32_t ms);
static void waitWithSignalLogs(unsigned long totalMs);
static bool netAvailHigh();
static bool waitForNetAvailAndMinCSQ(unsigned long maxWaitMs, int minCSQ, int stableSamples);
static void pixelPowerOn();
static void pixelPowerOff();
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

static void waitWithSignalLogs(const unsigned long totalMs) {
  static constexpr unsigned long kSignalSampleMs = CFG_NA_SAMPLE_MS;
  const unsigned long start = millis();
  const unsigned long end = start + totalMs;
  unsigned long nextSampleAt = start;
  while (true) {
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
    if (pixelMode == MODE_WAITING) {
      const unsigned long nextBlinkAt = lastBlinkToggle + kWaitBlinkMs;
      if (nextBlinkAt < nextEvent) nextEvent = nextBlinkAt;
    }
    if (nextSampleAt < nextEvent) nextEvent = nextSampleAt;
    if (nextEvent > now) lowPowerDelayMs(nextEvent - now);
  }
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
  if (stableSamples < 1) stableSamples = 1;

  while (millis() - start < maxWaitMs) {
    updateWaitingBlink();
    const unsigned long now = millis();

    if (now < nextSampleAt) {
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

// ---------- NeoPixel power gating ----------
static void pixelPowerOn() {
#if defined(NEOPIXEL_POWER)
  if (ENABLE_NEOPIXEL_POWER_GATING) {
    digitalWrite(NEOPIXEL_PWR, HIGH);
    // tiny settle helps avoid first-frame weirdness on some boards
    delayMicroseconds(200);
  }
#endif
}

static void pixelPowerOff() {
#if defined(NEOPIXEL_POWER)
  if (ENABLE_NEOPIXEL_POWER_GATING) {
    // Turn LED off *then* remove power.
    pixels.fill(C_OFF());
    pixels.show();
    digitalWrite(NEOPIXEL_PWR, LOW);
  }
#endif
}

// ---------- Pixel helpers ----------
static void pixelShowColor(const uint32_t c) {
  pixelPowerOn();
  pixels.fill(c);
  pixels.show();
}

static void pixelSetMode(const PixelMode mode) {
  pixelMode = mode;
  switch (mode) {
    case MODE_IDLE:
      pixelShowColor(C_OFF());
      pixelPowerOff();
      break;
    case MODE_WAITING:
      waitBlinkOn = true;
      lastBlinkToggle = millis();
      pixelShowColor(C_YELLOW());
      break;
    case MODE_FAIL:
      pixelShowColor(C_RED());
      break;
    case MODE_SUCCESS:
      pixelShowColor(C_GREEN());
      break;
  }
}

// Library callback (called repeatedly during modem work)
// Blink yellow while waiting.
bool ISBDCallback() {
  updateWaitingBlink();
  return true; // never cancel
}

static void updateWaitingBlink() {
  if (pixelMode != MODE_WAITING) return;
  if (const unsigned long now = millis(); now - lastBlinkToggle >= kWaitBlinkMs) {
    waitBlinkOn = !waitBlinkOn;
    pixelShowColor(waitBlinkOn ? C_YELLOW() : C_OFF());
    lastBlinkToggle = now;
  }
}

static void waitForSerialIfEnabled() {
  if constexpr (WAIT_FOR_USB_SERIAL_MS == 0) return;
  const unsigned long start = millis();
  while (!SerialMon && (millis() - start < WAIT_FOR_USB_SERIAL_MS)) { delay(10); }
}

// ---------- Modem helpers ----------
static void applyModemSettings() {
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);
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

  // NeoPixel init + power pin
#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_PWR, OUTPUT);
  // start with NeoPixel power OFF
  digitalWrite(NEOPIXEL_PWR, LOW);
#endif
  pixels.begin();
  pixels.setBrightness(CFG_NEOPIXEL_BRIGHTNESS);
  pixelSetMode(MODE_IDLE);

  // RockBLOCK UART
  Serial1.begin(19200);

  // If sleep pin is wired, keep modem asleep/off until we actually need to send.
#if (PIN_ISBD_SLEEP >= 0)
  pinMode(PIN_ISBD_SLEEP, OUTPUT);
  digitalWrite(PIN_ISBD_SLEEP, LOW);
#endif

#if (PIN_ISBD_NA >= 0)
  pinMode(PIN_ISBD_NA, INPUT);
#endif

#if !IF_QUIET
  SerialMon.println("KB2040 + RockBLOCK + NeoPixel (WAIT=blink yellow, FAIL=red, SUCCESS=green)");
  SerialMon.println("Press D9 (ALERT) or D8 (SOS) to send.\n\n\n");

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
  // End SUCCESS hold
  if (pixelMode == MODE_SUCCESS && successUntil != 0 && millis() >= successUntil) {
    pixelSetMode(MODE_IDLE);
    successUntil = 0;
  }

  // Read buttons with light debounce
  const bool curAlert = digitalRead(BTN_ALERT);
  const bool curSOS   = digitalRead(BTN_SOS);

  if (const unsigned long now = millis(); now - lastBounceMs > 30) {
    if (edgePressed(curAlert, lastAlert)) {
#if !IF_QUIET
      SerialMon.println("ALERT button pressed.");
#endif
      sendStartMs = millis();
      bool firstAttempt = true;
      while (true) {
        const bool ok = sendTextWithIndicators("ALERT", false, firstAttempt);
        firstAttempt = false;
        if (ok) break;
        // If we got here, send failed. Compute adaptive delay (often longer for no-network).
        const unsigned long delayMs = computeRetryDelayMs(lastSendErr, lastMoStatus, lastSawSBDIX);
#if !IF_QUIET
        SerialMon.println("Retrying after delay...\n\n");
#endif
        const bool showFail = (sendStartMs != 0) && ((millis() - sendStartMs) >= CFG_FAILURE_GRACE_MS);
        pixelSetMode(showFail ? MODE_FAIL : MODE_WAITING);
        waitWithSignalLogs(delayMs);
      }
    }

    if (edgePressed(curSOS, lastSOS)) {
#if !IF_QUIET
      SerialMon.println("SOS button pressed.");
#endif
      sendStartMs = millis();
      bool firstAttempt = true;
      while (true) {
        const bool ok = sendTextWithIndicators("SOS", true, firstAttempt);
        firstAttempt = false;
        if (ok) break;
        const unsigned long delayMs = computeRetryDelayMs(lastSendErr, lastMoStatus, lastSawSBDIX);
#if !IF_QUIET
        SerialMon.println("Retrying after delay...\n\n");
#endif
        const bool showFail = (sendStartMs != 0) && ((millis() - sendStartMs) >= CFG_FAILURE_GRACE_MS);
        pixelSetMode(showFail ? MODE_FAIL : MODE_WAITING);
        waitWithSignalLogs(delayMs);
      }
    }
    lastBounceMs = now;
  }

  // Idle: keep the MCU from spinning hot.
  if (pixelMode == MODE_IDLE) {
    lowPowerDelayMs(CFG_IDLE_POLL_MS);
  }
}
