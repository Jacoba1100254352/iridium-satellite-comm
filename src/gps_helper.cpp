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
static constexpr unsigned long kGpsFixTimeoutMs = CFG_GPS_SEND_FIX_TIMEOUT_MS;
static volatile unsigned long lastPpsMs = 0;
static volatile bool ppsActive = false;
static bool gnssInitialized = false;
static bool waitingMsgShown = false;

// Interrupt handler for PPS signal
/**
 * Records the latest GNSS PPS pulse so the rest of the firmware can tell
 * whether the receiver appears to be actively timing-valid in the background.
 */
void ppsInterrupt() {
    lastPpsMs = millis();
    ppsActive = true;
}

/**
 * Reports whether a recent PPS pulse has been seen, which acts as a coarse
 * "GNSS is alive and recently timed" hint for diagnostics.
 */
bool hasBackgroundFix() {
    if (!ppsActive) return false;
    if (millis() - lastPpsMs > 1500) {
        ppsActive = false;
        return false;
    }
    return true;
}

/**
 * Copies the current GNSS fix and timestamp into the retained last-known-fix
 * structure so sends can fall back to useful location metadata immediately.
 */
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

/**
 * Resets and initializes the GNSS receiver, configures its I2C transport, and
 * attaches the PPS interrupt used for low-cost background fix monitoring.
 */
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

/**
 * Polls GNSS state opportunistically in the background so a later send can use
 * either a fresh fix or the most recent retained location without blocking.
 */
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

/**
 * Appends either a fresh or last-known GNSS fix to the outbound message text
 * using the compact operator-facing payload format used by the project.
 */
static void appendFixToBuffer(char* buffer, const GpsFix& fix, bool isLKG) {
    const size_t currentLen = strlen(buffer);
    snprintf(buffer + currentLen, 120 - currentLen,
             "%s Lat:%.5f Lon:%.5f @%02d:%02d:%02dZ",
             isLKG ? " LKG" : "",
             fix.lat, fix.lon, fix.hour, fix.minute, fix.second);
}

/**
 * Adds GNSS data to the outbound payload without unnecessarily blocking the
 * send path: use a fresh fix if one is already available, otherwise fall back
 * to last-known-good data or NOFIX, with optional bounded search if enabled.
 */
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

    if (lastKnownFix.valid) {
        appendFixToBuffer(outputBuffer, lastKnownFix, true);
#if !IF_QUIET
        SerialMon.print(F("GPS send uses LKG fix immediately: "));
        SerialMon.println(outputBuffer);
#endif
        return;
    }

    if (kGpsFixTimeoutMs == 0) {
        const size_t currentLen = strlen(outputBuffer);
        snprintf(outputBuffer + currentLen, 120 - currentLen, " NOFIX");
#if !IF_QUIET
        SerialMon.println(F("GPS send skips fresh-fix wait; no prior fix available."));
#endif
        return;
    }

    // 2. Search for a new fix (timeout is configurable; 0 disables blocking search)
    pixelSetMode(MODE_GPS_SEARCH);
#if !IF_QUIET
    SerialMon.print(F("Searching for new GPS fix ("));
    SerialMon.print(kGpsFixTimeoutMs / 1000UL);
    SerialMon.println(F("s timeout)..."));
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
        snprintf(outputBuffer + currentLen, 120 - currentLen, " NOFIX");
#if !IF_QUIET
        SerialMon.println(F("GPS Timeout. No previous fix available."));
#endif
    }
}
