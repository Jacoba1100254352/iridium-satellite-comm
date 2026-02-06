#ifndef GPS_HELPER_H
#define GPS_HELPER_H

#include <Arduino.h>

void setupGps();
void processGps();
void attemptGpsFix(char* outputBuffer, bool (*cancelCheck)());
bool hasBackgroundFix();

#endif // GPS_HELPER_H
