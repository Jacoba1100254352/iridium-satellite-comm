#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <IridiumSBD.h>
#include "../include/config.h"
#include "../include/print_functions.h"

// =========================
// Buttons (active-LOW to GND)
// =========================
static constexpr int BTN_ALERT = 9;   // D9 → GND sends "ALERT"
static constexpr int BTN_SOS   = 8;   // D8 → GND sends "SOS"

// =========================
// Timing
// =========================
static constexpr unsigned long RETRY_DELAY_MS   = 10000UL; // keep FAIL shown during this delay
static constexpr unsigned long SUCCESS_HOLD_MS  = 10000UL; // green hold after success
static constexpr unsigned long WAIT_BLINK_MS    = 250UL;   // yellow blink period

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
static volatile PixelMode pixelMode = MODE_IDLE;
static volatile bool waitBlinkOn = false;
static unsigned long lastBlinkToggle = 0;
static unsigned long successUntil = 0;

// Debounce
static bool lastAlert = true; // pullup idle HIGH
static bool lastSOS   = true;
static unsigned long lastBounceMs = 0;

// =========================
// RockBLOCK on Serial1 (UART0: TX=D0, RX=D1)
// =========================
// Optional: define sleep and ring pins for power and wake control.  If you attach
// these pins to your microcontroller and RockBLOCK, uncomment the definitions
// below and set the numbers to match your wiring.  The sleep pin should connect
// to the RockBLOCK ON_OFF (or SLP) line, and the ring pin should connect to
// the RockBLOCK RI output.
// static constexpr int PIN_ISBD_SLEEP = 7;   // microcontroller pin for ON_OFF / SLP
// static constexpr int PIN_ISBD_RI    = 6;   // microcontroller pin for RI (active low)
//
// When both pins are defined, construct the modem with:
// IridiumSBD modem(Serial1, PIN_ISBD_SLEEP, PIN_ISBD_RI);
// and then call modem.enableRingAlerts(true) in setup().
//
// In the current configuration no extra pins are attached, so use the basic constructor:
IridiumSBD modem(Serial1);

#if DIAGNOSTICS
// void ISBDConsoleCallback(IridiumSBD *d, const char c) { SerialMon.write(c); }
void ISBDConsoleCallback(IridiumSBD *d, const char c) {
  // Only echo raw characters in VERBOSE
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
static bool sendTextWithIndicators(const char *text);

// ---------- Pixel helpers ----------
static void pixelShowColor(const uint32_t c) {
  pixels.fill(c);
  pixels.show();
}

static void pixelSetMode(const PixelMode mode) {
  pixelMode = mode;
  switch (mode) {
    case MODE_IDLE:
      pixelShowColor(C_OFF());
      break;
    case MODE_WAITING:
      waitBlinkOn = false;
      lastBlinkToggle = millis();
      pixelShowColor(C_OFF());
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
  if (pixelMode == MODE_WAITING) {
    if (const unsigned long now = millis(); now - lastBlinkToggle >= WAIT_BLINK_MS) {
      waitBlinkOn = !waitBlinkOn;
      pixelShowColor(waitBlinkOn ? C_YELLOW() : C_OFF());
      lastBlinkToggle = now;
    }
  }
  return true; // never cancel
}

static void waitForSerial(unsigned long ms = 4000) {
  const unsigned long start = millis();
  while (!SerialMon && (millis() - start < ms)) { delay(10); }
}

void setup() {
  // Buttons: active-LOW to GND
  pinMode(BTN_ALERT, INPUT_PULLUP);
  pinMode(BTN_SOS,   INPUT_PULLUP);

  // USB Serial
  SerialMon.begin(115200);
  waitForSerial();

  // NeoPixel power (if present) and init
  #if defined(NEOPIXEL_POWER)
    pinMode(NEOPIXEL_PWR, OUTPUT);
    digitalWrite(NEOPIXEL_PWR, HIGH);
  #endif
  pixels.begin();
  // pixels.setBrightness(50);
  pixels.setBrightness(8);    // Dim for battery conservation
  pixelSetMode(MODE_IDLE);

  // RockBLOCK UART
  Serial1.begin(19200);  // D0/D1 default UART0

  SerialMon.println("KB2040 + RockBLOCK + NeoPixel (WAIT=blink yellow, FAIL=red, SUCCESS=green)");

  int err = modem.begin();
  if (err != ISBD_SUCCESS) {
    SerialMon.print("modem.begin() failed, err="); SerialMon.println(err);
    if (err == ISBD_NO_MODEM_DETECTED) SerialMon.println("No modem detected.");
    pixelSetMode(MODE_FAIL);
    while (true) { delay(1000); }
  }

  char fw[16] = {};
  err = modem.getFirmwareVersion(fw, sizeof(fw));
  if (err == ISBD_SUCCESS) {
    SerialMon.print("FW: "); SerialMon.println(fw);
  }

  int csq = -1;
  err = modem.getSignalQuality(csq);
  if (err == ISBD_SUCCESS) {
    SerialMon.print("Signal quality (0-5): "); SerialMon.println(csq);
  }

  /***   POWER EFFICIENCY SETTINGS   ***/
  // Adjust modem timeouts and power profile for battery efficiency.  These settings
  // shorten timeouts so the radio does not stay powered longer than necessary.  Feel
  // free to tune these values based on your environment.
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);
  modem.adjustATTimeout(10);
  modem.adjustSendReceiveTimeout(120);
  modem.adjustStartupTimeout(60);
  modem.adjustSBDSessionTimeout(180);
  // Enable ring alerts when a ring indicator pin is attached.
  // modem.enableRingAlerts(true);
  // Keep the MSSTM workaround enabled.
  modem.useMSSTMWorkaround(true);

  /// Optional: enable diagnostic console output
#if IF_VERBOSE
  SerialMon.println();
  SerialMon.println("SBDIX fields explanation:");
  SerialMon.println("  MO-status:  Mobile Originated status (e.g., 0=success, 32=no network service)");
  SerialMon.println("  MOMSN:      Mobile Originated Message Sequence Number (increments with each send)");
  SerialMon.println("  MT-status:  Mobile Terminated status (0=no message, 1=message received, 2=error)");
  SerialMon.println("  MTMSN:      Mobile Terminated Message Sequence Number (for the received message)");
  SerialMon.println("  MT-length:  Length in bytes of the received Mobile Terminated message");
  SerialMon.println("  MT-queued:  Number of pending Mobile Terminated messages still waiting on the server");
  SerialMon.println();

  // Hook up console and diag callbacks
  printModemGlossaryOnce();
#endif

  SerialMon.println("Press D9 (ALERT) or D8 (SOS) to send.\n\n\n");
}

// Build small MO payload: [len8][ASCII bytes...], perform send+receive, and drive NeoPixel states.
static bool sendTextWithIndicators(const char *text) {
  size_t len = strlen(text);
  if (len > 110) len = 110;
  uint8_t mo[1 + 110] = {};
  mo[0] = static_cast<uint8_t>(len);
  memcpy(&mo[1], text, len);

  uint8_t mt[270];
  size_t mtLen = sizeof(mt);

  // Start WAITING (blink yellow)
  SerialMon.print("Sending \""); SerialMon.print(text); SerialMon.println("\"...");
  pixelSetMode(MODE_WAITING);

  // Send/receive SBD
  const int err = modem.sendReceiveSBDBinary(mo, 1 + len, mt, mtLen);
  if (gSBDIXSeen) {
// #ifdef IF_COMPACT
//     printSBDIXCompact();
// #elifdef IF_VERBOSE
//     printSBDIXVerbose();
// #else
//     // No extra printing
// #endif
//     if (gMOStatus == 32) SerialMon.println("Hint: No network service — move to clear sky; try for CSQ >= 2.");
    gSBDIXSeen = false;
  }

  if (err != ISBD_SUCCESS) {
    /***   ON FAILURE   ***/
    SerialMon.print("SBD send/receive failed, err=");
    SerialMon.print(err);
    SerialMon.print(".\tReason:");
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

    pixelSetMode(MODE_FAIL); // red solid during caller's retry delay
    return false;

    /***   ON SUCCESS   ***/
    // ReSharper disable once CppRedundantElseKeywordInsideCompoundStatement
  } else {
    // Print SBDIX status
    SerialMon.println("Send OK.");
    if (mtLen > 0) {
      SerialMon.print("Received "); SerialMon.print(mtLen); SerialMon.println(" byte(s):");
      for (size_t i = 0; i < mtLen; ++i) {
        if (const uint8_t b = mt[i]; b >= 32 && b <= 126) SerialMon.write(static_cast<char>(b)); else SerialMon.print(".");
      }
      SerialMon.println();
    } else {
      SerialMon.println("No MT message queued.");
    }

    // SUCCESS: green for 10s (non-blocking), then off
    pixelSetMode(MODE_SUCCESS);
    successUntil = millis() + SUCCESS_HOLD_MS;
    return true;
  }
}

// edge detection for active-LOW buttons
static bool edgePressed(const bool current, bool &last) {
  const bool pressed = (last == true && current == false); // HIGH->LOW
  last = current;
  return pressed;
}

void loop() {
  // Manage SUCCESS hold duration
  if (pixelMode == MODE_SUCCESS && successUntil != 0 && millis() >= successUntil) {
    pixelSetMode(MODE_IDLE);
    successUntil = 0;
  }

  // Read buttons with light debounce
  const bool curAlert = digitalRead(BTN_ALERT);
  const bool curSOS   = digitalRead(BTN_SOS);

  if (const unsigned long now = millis(); now - lastBounceMs > 30) {
    if (edgePressed(curAlert, lastAlert)) {
      SerialMon.println("ALERT button pressed.");
      while (true) {
        if (sendTextWithIndicators("ALERT")) break; // if OK (success)
        SerialMon.println("Retrying after delay...\n\n");
        const unsigned long t0 = millis();
        pixelSetMode(MODE_FAIL); // red during wait
        while (millis() - t0 < RETRY_DELAY_MS) { delay(10); }
      }
    }

    if (edgePressed(curSOS, lastSOS)) {
      SerialMon.println("SOS button pressed.");
      while (true) {
        if (sendTextWithIndicators("SOS")) break;   // if OK (success)
        SerialMon.println("Retrying after delay...\n\n");
        const unsigned long t0 = millis();
        pixelSetMode(MODE_FAIL);
        while (millis() - t0 < RETRY_DELAY_MS) { delay(10); }
      }
    }
    lastBounceMs = now;
  }

  // If mode is WAITING, the yellow blink is handled inside ISBDCallback()
}
