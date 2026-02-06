#ifndef GPS_HELPER_H
#define GPS_HELPER_H

#include <Arduino.h>

void setupGps();
void attemptGpsFix(char* outputBuffer, bool (*cancelCheck)());

#endif // GPS_HELPER_H
