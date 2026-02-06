#include "../include/gps_helper.h"
#include "../include/led_helper.h"
#include "../include/config.h"
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Wire.h>

// =========================
// GNSS Object & Storage
// =========================
SFE_UBLOX_GNSS myGNSS;

struct GpsFix {
    float lat;
    float lon;
    uint16_t year;
    uint8_t month, day, hour, minute, second;
    bool valid = false;
};

static GpsFix lastKnownFix;
static unsigned long lastBackgroundPollMs = 0;

static constexpr unsigned long GPS_BACKGROUND_POLL_MS = 5 * 60 * 1000UL; // 5 minutes
static constexpr unsigned long kGpsFixTimeoutMs = 15000;
static volatile unsigned long lastPpsMs = 0;
static volatile bool ppsActive = false;
static bool gnssInitialized = false;
static bool waitingMsgShown = false;

// Interrupt handler for PPS signal
void ppsInterrupt() {
    lastPpsMs = millis();
    ppsActive = true;
}

bool hasBackgroundFix() {
    if (!ppsActive) return false;
    if (millis() - lastPpsMs > 1500) {
        ppsActive = false;
        return false;
    }
    return true;
}

static void updateLastKnown() {
    if (myGNSS.getGnssFixOk()) {
        lastKnownFix.lat = static_cast<float>(myGNSS.getLatitude()) / 10000000.0f;
        lastKnownFix.lon = static_cast<float>(myGNSS.getLongitude()) / 10000000.0f;
        lastKnownFix.year = myGNSS.getYear();
        lastKnownFix.month = myGNSS.getMonth();
        lastKnownFix.day = myGNSS.getDay();
        lastKnownFix.hour = myGNSS.getHour();
        lastKnownFix.minute = myGNSS.getMinute();
        lastKnownFix.second = myGNSS.getSecond();
        lastKnownFix.valid = true;

#if !IF_QUIET
        SerialMon.print(F("GNSS Fix Obtained: "));
        SerialMon.print(lastKnownFix.lat, 6);
        SerialMon.print(F(", "));
        SerialMon.print(lastKnownFix.lon, 6);
        SerialMon.print(F(" @ "));
        SerialMon.print(lastKnownFix.year);
        SerialMon.print(F("-"));
        SerialMon.print(lastKnownFix.month);
        SerialMon.print(F("-"));
        SerialMon.print(lastKnownFix.day);
        SerialMon.print(F(" "));
        if (lastKnownFix.hour < 10) SerialMon.print('0');
        SerialMon.print(lastKnownFix.hour);
        SerialMon.print(F(":"));
        if (lastKnownFix.minute < 10) SerialMon.print('0');
        SerialMon.print(lastKnownFix.minute);
        SerialMon.print(F(":"));
        if (lastKnownFix.second < 10) SerialMon.print('0');
        SerialMon.print(lastKnownFix.second);
        SerialMon.println(F(" UTC"));
#endif
    }
}

void setupGps() {
    // 1. Handle Hardware Reset
#if (PIN_GPS_RST >= 0)
    SerialMon.println(F("Resetting GNSS..."));
    pinMode(PIN_GPS_RST, OUTPUT);
    digitalWrite(PIN_GPS_RST, LOW);
    delay(100);
    digitalWrite(PIN_GPS_RST, HIGH);
    delay(2000); 
#endif

    // 2. Initialize I2C with CUSTOM PINS
    Wire.setSDA(PIN_GPS_SDA);
    Wire.setSCL(PIN_GPS_SCL);
    Wire.begin();

    // 3. Initialize GNSS Module
    // Note: Debugging is enabled here. If hex output is too noisy, 
    // you can comment out enableDebugging.
    // myGNSS.enableDebugging(SerialMon);

    int retries = 3;
    while (retries > 0) {
        if (myGNSS.begin()) {
            myGNSS.setI2COutput(COM_TYPE_UBX);
            myGNSS.saveConfiguration();
            gnssInitialized = true;
            SerialMon.println(F("GNSS module detected."));
            break;
        } else {
            SerialMon.print(F("GNSS connection attempt failed. Retries left: "));
            SerialMon.println(retries - 1);
            delay(1000);
            retries--;
        }
    }

    if (!gnssInitialized) {
        SerialMon.println(F("CRITICAL: GNSS module NOT detected. Check wiring."));
    }

    // 4. Configure PPS Interrupt
#if (PIN_GPS_PPS >= 0)
    pinMode(PIN_GPS_PPS, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), ppsInterrupt, RISING);
#endif

    // 5. External Interrupt pin (EINT)
#if (PIN_GPS_EINT >= 0)
    pinMode(PIN_GPS_EINT, INPUT);
#endif
}

void processGps() {
    if (!gnssInitialized) return;

    // Poll every 2 seconds if no fix yet (to show fix ASAP), then use background interval.
    unsigned long pollInterval = lastKnownFix.valid ? GPS_BACKGROUND_POLL_MS : 2000;

    if (millis() - lastBackgroundPollMs > pollInterval) {
        if (myGNSS.getGnssFixOk()) {
            updateLastKnown();
            waitingMsgShown = false; // Reset for if fix is lost
        } else {
            if (!lastKnownFix.valid && !waitingMsgShown) {
#if !IF_QUIET
                SerialMon.println(F("Waiting for a GNSS fix."));
#endif
                waitingMsgShown = true;
            }
        }
        lastBackgroundPollMs = millis();
    }
}

static void appendFixToBuffer(char* buffer, const GpsFix& fix, bool isLKG) {
    const size_t currentLen = strlen(buffer);
    snprintf(buffer + currentLen, 120 - currentLen,
             "%s Lat:%.5f Lon:%.5f @%02d:%02d:%02dZ",
             isLKG ? " LKG" : "",
             fix.lat, fix.lon, fix.hour, fix.minute, fix.second);
}

void attemptGpsFix(char* outputBuffer, bool (*cancelCheck)()) {
    if (!gnssInitialized) {
#if !IF_QUIET
        SerialMon.println(F("GPS skip: Module not initialized."));
#endif
        return;
    }

    // 1. Try for an instant fresh fix
    if (myGNSS.getGnssFixOk()) {
        updateLastKnown();
        appendFixToBuffer(outputBuffer, lastKnownFix, false);
        return;
    }

    // 2. Search for a new fix (15s timeout)
    pixelSetMode(MODE_GPS_SEARCH);
#if !IF_QUIET
    SerialMon.println(F("Searching for new GPS fix (15s timeout)..."));
#endif

    const unsigned long startSearch = millis();
    bool fixFound = false;

    while (millis() - startSearch < kGpsFixTimeoutMs) {
        if (cancelCheck && cancelCheck()) return;

        if (myGNSS.getGnssFixOk()) {
            updateLastKnown();
            fixFound = true;
            break;
        }

        static bool gpsBlink = false;
        static unsigned long lastGpsBlink = 0;
        if (millis() - lastGpsBlink > 200) {
            gpsBlink = !gpsBlink;
            pixelShowColor(gpsBlink ? C_BLUE() : C_OFF());
            lastGpsBlink = millis();
        }
        delay(10);
    }

    // 3. Output results
    if (fixFound) {
        appendFixToBuffer(outputBuffer, lastKnownFix, false);
    } else if (lastKnownFix.valid) {
        appendFixToBuffer(outputBuffer, lastKnownFix, true);
#if !IF_QUIET
        SerialMon.print(F("GPS Timeout. Using LKG fix: "));
        SerialMon.println(outputBuffer);
#endif
    } else {
        const size_t currentLen = strlen(outputBuffer);
        snprintf(outputBuffer + currentLen, 120 - currentLen, " No precise fix obtained");
#if !IF_QUIET
        SerialMon.println(F("GPS Timeout. No previous fix available."));
#endif
    }
}
