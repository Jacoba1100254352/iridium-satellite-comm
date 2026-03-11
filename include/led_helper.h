#ifndef LED_HELPER_H
#define LED_HELPER_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

enum PixelMode : uint8_t { MODE_IDLE, MODE_WAITING, MODE_RETRY_WAIT, MODE_FAIL, MODE_SUCCESS, MODE_GPS_SEARCH };

void setupLeds();
void pixelShowColor(uint32_t c);
void pixelSetMode(PixelMode mode);
void updateWaitingBlink();
void pixelPowerOn();
void pixelPowerOff();
PixelMode getPixelMode();
unsigned long getLastBlinkToggle();

// Quick color helpers
uint32_t C_RED();
uint32_t C_GREEN();
uint32_t C_YELLOW();
uint32_t C_ORANGE();
uint32_t C_BLUE();
uint32_t C_OFF();

#endif // LED_HELPER_H
