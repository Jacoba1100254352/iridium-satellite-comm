#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define BTN1_PIN    9
#define BTN2_PIN    8

// How many internal neopixels do we have? some boards have more than one!
#define NUMPIXELS         1

Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// When true → NeoPixel always on; when false → only on while a button is pressed
bool neoAlwaysOn = false;

void setup() {
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);

  pixels.begin();
  pixels.setBrightness(50);  // adjust brightness
  pixels.show();              // initialize all pixels “off”

  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("KB2040 NeoPixel + Buttons test");
  Serial.println("Button1 on pin 9 → GND");
  Serial.println("Button2 on pin 8 → GND");
  Serial.println("NeoPixel on pin 4 (data line)");
  Serial.println("Change neoAlwaysOn = false to make NeoPixel only light with button press.");
}

void loop() {
  bool btn1 = (digitalRead(BTN1_PIN) == LOW);
  bool btn2 = (digitalRead(BTN2_PIN) == LOW);

  if (btn1) {
    Serial.println("Button1 pressed!");
  }
  if (btn2) {
    Serial.println("Button2 pressed!");
  }

  bool shouldLight = neoAlwaysOn || btn1 || btn2;

  if (shouldLight) {
    // e.g., green when on
    pixels.fill(pixels.Color(0, 255, 0));
  } else {
    pixels.fill(pixels.Color(0, 0, 0)); // off
  }

  pixels.show();

  delay(100); // simple debounce / loop pace
}
