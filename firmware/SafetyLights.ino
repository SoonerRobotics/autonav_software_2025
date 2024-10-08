#include <Adafruit_NeoPixel.h>


static const int MosfetPIN = 17; // Pin for Data Input
static const int DIPIN = 16; // Pin for Mosfet gate
static const int NUMPixels = 25; // Define Number of Pixels
static const int DEFAULT_BRIGHTNESS = 55; 


Adafruit_NeoPixel strip(NUMPixels, DIPIN, NEO_GRB);
int current_color = strip.Color(255,255,255);
int current_brightness = DEFAULT_BRIGHTNESS;

void setup() {  // put your setup code here, to run once:

pinMode(LED_BUILTIN, OUTPUT);

pinMode(DIPIN, OUTPUT);
digitalWrite(DIPIN, HIGH);
pinMode(MosfetPIN, OUTPUT);
digitalWrite(LED_BUILTIN, HIGH);


strip.begin();
strip.show();
strip.setBrightness(current_brightness);
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(MosfetPIN, LOW);


for(int i=0; i<NUMPixels; i++) {
  strip.setPixelColor(i, strip.Color(255,255,255));
  strip.show();
  delay(1000);

}
 strip.clear();
delay(1000);
}
