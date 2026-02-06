#include "../include/gps_helper.h"
#include "../include/led_helper.h"
#include "../include/config.h"
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Wire.h>

// =========================
// GNSS Object
// =========================
SFE_UBLOX_GNSS myGNSS;

static constexpr unsigned long kGpsFixTimeoutMs = 15000; // 15 seconds to find satellites

void setupGps() {
    // Initialize I2C for GNSS
    Wire.begin();
}

void attemptGpsFix(char* outputBuffer, bool (*cancelCheck)()) {
    pixelSetMode(MODE_GPS_SEARCH); // Solid Blue

    // Attempt to connect to GNSS
    if (myGNSS.begin() == false) {
        SerialMon.println(F("u-blox GNSS not detected. Skipping GPS."));
        return;
    }

    myGNSS.setI2COutput(COM_TYPE_UBX); // Minimize I2C chatter
    myGNSS.saveConfiguration();

    const unsigned long startSearch = millis();
    bool fixFound = false;
    int32_t lat = 0;
    int32_t lon = 0;

    SerialMon.println(F("Searching for GPS Fix..."));

    while (millis() - startSearch < kGpsFixTimeoutMs) {
        // Allow user to cancel (hold both buttons) during GPS search
        if (cancelCheck && cancelCheck()) return;

        // Check for fix
        if (myGNSS.getGnssFixOk()) {
            lat = myGNSS.getLatitude();
            lon = myGNSS.getLongitude();
            fixFound = true;
            break;
        }

        // Blink Blue/Off to show activity
        static bool gpsBlink = false;
        static unsigned long lastGpsBlink = 0;
        if (millis() - lastGpsBlink > 200) {
            gpsBlink = !gpsBlink;
            pixelShowColor(gpsBlink ? C_BLUE() : C_OFF());
            lastGpsBlink = millis();
        }
        delay(10);
    }

    if (fixFound) {
        // Format: " Lat: 34.12345 Lon: -84.12345"
        // Library returns degrees * 10^7
        const float fLat = static_cast<float>(lat) / 10000000.0f;
        const float fLon = static_cast<float>(lon) / 10000000.0f;
        // Append to existing text
        const size_t currentLen = strlen(outputBuffer);
        snprintf(outputBuffer + currentLen, 120 - currentLen, " Lat:%.5f Lon:%.5f", fLat, fLon);

        SerialMon.print(F("Fix found: "));
        SerialMon.println(outputBuffer);
    } else {
        SerialMon.println(F("GPS Timeout. Sending without coordinates."));
    }
}
