#include "../include/led_helper.h"
#include "../include/config.h"

// =========================
// NeoPixel (KB2040 onboard)
// =========================
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

#if defined(NEOPIXEL_POWER)
static const int NEOPIXEL_PWR = NEOPIXEL_POWER;
#endif

// State
static PixelMode pixelMode = MODE_IDLE;
static bool waitBlinkOn = false;
static unsigned long lastBlinkToggle = 0;

static constexpr unsigned long kWaitBlinkMs = CFG_WAIT_BLINK_MS;
static constexpr unsigned long kRetryWaitBlinkMs = CFG_RETRY_WAIT_BLINK_MS;

void setupLeds() {
#if defined(NEOPIXEL_POWER)
    pinMode(NEOPIXEL_PWR, OUTPUT);
    // start with NeoPixel power OFF
    digitalWrite(NEOPIXEL_PWR, LOW);
#endif
    pixels.begin();
    pixels.setBrightness(CFG_NEOPIXEL_BRIGHTNESS);
    pixelSetMode(MODE_IDLE);
}

void pixelPowerOn() {
#if defined(NEOPIXEL_POWER)
    if (ENABLE_NEOPIXEL_POWER_GATING) {
        digitalWrite(NEOPIXEL_PWR, HIGH);
        // tiny settle helps avoid first-frame weirdness on some boards
        delayMicroseconds(200);
    }
#endif
}

void pixelPowerOff() {
#if defined(NEOPIXEL_POWER)
    if (ENABLE_NEOPIXEL_POWER_GATING) {
        pixels.fill(C_OFF());
        pixels.show();
        digitalWrite(NEOPIXEL_PWR, LOW);
    }
#endif
}

void pixelShowColor(const uint32_t c) {
    pixelPowerOn();
    pixels.fill(c);
    pixels.show();
}

void pixelSetMode(const PixelMode mode) {
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
        case MODE_RETRY_WAIT:
            waitBlinkOn = true;
            lastBlinkToggle = millis();
            pixelShowColor(C_ORANGE());
            break;
        case MODE_FAIL:
            pixelShowColor(C_RED());
            break;
        case MODE_SUCCESS:
            pixelShowColor(C_GREEN());
            break;
        case MODE_GPS_SEARCH:
            pixelShowColor(C_BLUE());
            break;
    }
}

void updateWaitingBlink() {
    const unsigned long blinkMs = (pixelMode == MODE_RETRY_WAIT) ? kRetryWaitBlinkMs : kWaitBlinkMs;
    if (pixelMode != MODE_WAITING && pixelMode != MODE_RETRY_WAIT) return;
    if (const unsigned long now = millis(); now - lastBlinkToggle >= blinkMs) {
        waitBlinkOn = !waitBlinkOn;
        const uint32_t activeColor = (pixelMode == MODE_RETRY_WAIT) ? C_ORANGE() : C_YELLOW();
        pixelShowColor(waitBlinkOn ? activeColor : C_OFF());
        lastBlinkToggle = now;
    }
}

PixelMode getPixelMode() {
    return pixelMode;
}

unsigned long getLastBlinkToggle() {
    return lastBlinkToggle;
}

// Quick color helpers
uint32_t C_RED()    { return Adafruit_NeoPixel::Color(255, 0,   0  ); }
uint32_t C_GREEN()  { return Adafruit_NeoPixel::Color(0,   255, 0  ); }
uint32_t C_YELLOW() { return Adafruit_NeoPixel::Color(255, 200, 0  ); }
uint32_t C_ORANGE() { return Adafruit_NeoPixel::Color(255, 80,  0  ); }
uint32_t C_BLUE()   { return Adafruit_NeoPixel::Color(0,   0,   255); }
uint32_t C_OFF()    { return Adafruit_NeoPixel::Color(0,   0,   0  ); }
