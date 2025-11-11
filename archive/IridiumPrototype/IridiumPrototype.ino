#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <IridiumSBD.h>

#define DIAGNOSTICS true
#define SerialMon Serial

// =========================
// Buttons (active-LOW to GND)
// =========================
static const int BTN_ALERT = 9;   // D9 → GND sends "ALERT"
static const int BTN_SOS   = 8;   // D8 → GND sends "SOS"

// =========================
// Timing
// =========================
static const unsigned long RETRY_DELAY_MS   = 10000UL; // keep FAIL shown during this delay
static const unsigned long SUCCESS_HOLD_MS  = 10000UL; // green hold after success
static const unsigned long WAIT_BLINK_MS    = 250UL;   // yellow blink period

// =========================
// NeoPixel (KB2040 onboard)
// =========================
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#if defined(NEOPIXEL_POWER)
  static const int NEOPIXEL_PWR = NEOPIXEL_POWER;
#endif

// Quick color helpers
static inline uint32_t C_RED()    { return pixels.Color(255, 0,   0  ); }
static inline uint32_t C_GREEN()  { return pixels.Color(0,   255, 0  ); }
static inline uint32_t C_YELLOW() { return pixels.Color(255, 200, 0  ); }
static inline uint32_t C_OFF()    { return pixels.Color(0,   0,   0  ); }

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
IridiumSBD modem(Serial1);

#if DIAGNOSTICS
void ISBDConsoleCallback(IridiumSBD *d, char c) { SerialMon.write(c); }
void ISBDDiagsCallback(IridiumSBD *d, char c)    { SerialMon.write(c); }
#endif

// Forward decls
static void pixelShowColor(uint32_t c);
static void pixelSetMode(PixelMode mode);
static bool sendTextWithIndicators(const char *text);

// ---------- Pixel helpers ----------
static void pixelShowColor(uint32_t c) {
  pixels.fill(c);
  pixels.show();
}

static void pixelSetMode(PixelMode mode) {
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
    unsigned long now = millis();
    if (now - lastBlinkToggle >= WAIT_BLINK_MS) {
      waitBlinkOn = !waitBlinkOn;
      pixelShowColor(waitBlinkOn ? C_YELLOW() : C_OFF());
      lastBlinkToggle = now;
    }
  }
  return true; // never cancel
}

static void waitForSerial(unsigned long ms = 4000) {
  unsigned long start = millis();
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
  pixels.setBrightness(50);
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

  char fw[16] = {0};
  if ((err = modem.getFirmwareVersion(fw, sizeof(fw))) == ISBD_SUCCESS) {
    SerialMon.print("FW: "); SerialMon.println(fw);
  }

  int csq = -1;
  if ((err = modem.getSignalQuality(csq)) == ISBD_SUCCESS) {
    SerialMon.print("Signal quality (0-5): "); SerialMon.println(csq);
  }

  SerialMon.println("Press D9 (ALERT) or D8 (SOS) to send.");
}

// Add this helper function somewhere above sendTextWithIndicators()
static void printSBDIXStatus(int moStatus, int moMOMSN, int mtStatus,
                             int mtMSN, int mtLength, int mtQueued) {
  SerialMon.print("SBDIX result → ");
  SerialMon.print("MO-status="); SerialMon.print(moStatus);
  SerialMon.print(" (");
  switch (moStatus) {
    case 0:  SerialMon.print("Success"); break;
    case 1:  SerialMon.print("Partial success?"); break;
    case 32: SerialMon.print("No network service"); break;
    // You can add more cases based on your modem docs
    default: SerialMon.print("Unknown code"); break;
  }
  SerialMon.print("), MOMSN="); SerialMon.print(moMOMSN);

  SerialMon.print(", MT-status="); SerialMon.print(mtStatus);
  SerialMon.print(" (");
  switch (mtStatus) {
    case 0:  SerialMon.print("No MT message"); break;
    case 1:  SerialMon.print("MT message received"); break;
    case 2:  SerialMon.print("MT mailbox check error"); break;
    default: SerialMon.print("Unknown code"); break;
  }
  SerialMon.print("), MTMSN="); SerialMon.print(mtMSN);

  SerialMon.print(", MT-length="); SerialMon.print(mtLength);
  SerialMon.print(" bytes, MT-queued="); SerialMon.print(mtQueued);
  SerialMon.println(" messages waiting");
}

// Build small MO payload: [len8][ASCII bytes...], perform send+receive, and drive NeoPixel states.
static bool sendTextWithIndicators(const char *text) {
  size_t len = strlen(text);
  if (len > 110) len = 110;
  uint8_t mo[1 + 110] = {0};
  mo[0] = (uint8_t)len;
  memcpy(&mo[1], text, len);

  uint8_t mt[270];
  size_t mtLen = sizeof(mt);

  // Start WAITING (blink yellow)
  SerialMon.print("Sending \""); SerialMon.print(text); SerialMon.println("\"...");
  pixelSetMode(MODE_WAITING);

  int err = modem.sendReceiveSBDBinary(mo, (size_t)(1 + len), mt, mtLen);

  if (err != ISBD_SUCCESS) {
    SerialMon.print("SBD send/receive failed, err=");
    SerialMon.println(err);
    if (err == ISBD_SENDRECEIVE_TIMEOUT) SerialMon.println("Timeout.");
    pixelSetMode(MODE_FAIL); // red solid during caller's retry delay
    return false;
  }

  SerialMon.println("Send OK.");
  if (mtLen > 0) {
    SerialMon.print("Received "); SerialMon.print(mtLen); SerialMon.println(" byte(s):");
    for (size_t i = 0; i < mtLen; ++i) {
      uint8_t b = mt[i];
      if (b >= 32 && b <= 126) SerialMon.write((char)b); else SerialMon.print(".");
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

// edge detection for active-LOW buttons
static bool edgePressed(bool current, bool &last) {
  bool pressed = (last == true && current == false); // HIGH->LOW
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
  bool curAlert = digitalRead(BTN_ALERT);
  bool curSOS   = digitalRead(BTN_SOS);
  unsigned long now = millis();

  if (now - lastBounceMs > 30) {
    if (edgePressed(curAlert, lastAlert)) {
      SerialMon.println("ALERT button pressed.");
      while (true) {
        bool ok = sendTextWithIndicators("ALERT");
        if (ok) break;
        SerialMon.println("Retrying after delay...");
        unsigned long t0 = millis();
        pixelSetMode(MODE_FAIL); // red during wait
        while (millis() - t0 < RETRY_DELAY_MS) { delay(10); }
      }
    }

    if (edgePressed(curSOS, lastSOS)) {
      SerialMon.println("SOS button pressed.");
      while (true) {
        bool ok = sendTextWithIndicators("SOS");
        if (ok) break;
        SerialMon.println("Retrying after delay...");
        unsigned long t0 = millis();
        pixelSetMode(MODE_FAIL);
        while (millis() - t0 < RETRY_DELAY_MS) { delay(10); }
      }
    }
    lastBounceMs = now;
  }

  // If mode is WAITING, the yellow blink is handled inside ISBDCallback()
}