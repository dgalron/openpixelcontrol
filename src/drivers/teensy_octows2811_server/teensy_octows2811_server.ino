/* Driver for rendering lighting defined by the OPC protocol on a Teensy 3.x
   Microcontroller using the OctoWS2811 driver.

   Author: Daniel A. Galron
   Email: daniel.galron@gmail.com
   July 7, 2019

  Required Connections
  --------------------
    pin 2:  LED Strip #1    OctoWS2811 drives 8 LED Strips.
    pin 14: LED strip #2    All 8 are the same length.
    pin 7:  LED strip #3
    pin 8:  LED strip #4    A 100 to 220 ohm resistor should used
    pin 6:  LED strip #5    between each Teensy pin and the
    pin 20: LED strip #6    wire to the LED strip, to minimize
    pin 21: LED strip #7    high frequency ringining & noise.
    pin 5:  LED strip #8
    pin 15 & 16 - Connect together, but do not use
    pin 4:  Do not use
    pin 3:  Do not use as PWM.  Normal use is ok.
    pin 12: Frame Sync
*/

#include <OctoWS2811.h>
#include "opc.h"

// The actual arrangement of the LEDs connected to this Teensy 3.0 board.
// LED_HEIGHT *must* be a multiple of 8.  When 16, 24, 32 are used, each
// strip spans 2, 3, 4 rows.  LED_LAYOUT indicates the direction the strips
// are arranged.  If 0, each strip begins on the left for its first row,
// then goes right to left for its second row, then left to right,
// zig-zagging for each successive row.
#define LED_WIDTH      44   // number of LEDs horizontally
#define LED_HEIGHT     64   // number of LEDs vertically (must be multiple of 8)
#define LED_LAYOUT     0    // 0 = even rows left->right, 1 = even rows right->left

const int ledsPerStrip = LED_WIDTH * LED_HEIGHT / 8;

int buffer[LED_WIDTH*LED_HEIGHT];

DMAMEM int displayMemory[ledsPerStrip*6];
int drawingMemory[ledsPerStrip*6]; 
elapsedMicros elapsedUsecSinceLastFrameSync = 0;

const int config = WS2811_800kHz; // color config is on the PC side
OctoWS2811 leds(ledsPerStrip, displayMemory, drawingMemory, config);

void setup() {
  for (int i = 0; i < ledsPerStrip*6; ++i) {
    displayMemory[i] = 0;
    drawingMemory[i] = 0;
  }
  pinMode(12, INPUT_PULLUP); // Frame Sync
  Serial.setTimeout(50);
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  leds.begin();
  leds.show();
  digitalWrite(13, HIGH);
  if (leds.numPixels() != LED_WIDTH * LED_HEIGHT) {
    while(true) {
      Serial.print("Error ");
      Serial.print(leds.numPixels());
      Serial.print(" != ");
      Serial.println(LED_WIDTH * LED_HEIGHT);
      delay(1000);
    }
  }
}

void loop() {
  int bytesRead = Serial.readBytes((char *) buffer, sizeof(buffer));
  if (bytesRead == sizeof(buffer)) {
    for (int i = 0; i < 4; ++i) {
      char tbs[16];
      sprintf(tbs, "%20x ", buffer[i]); 
      Serial.print(tbs);
    }
    Serial.println();
    digitalWrite(12, HIGH);
    pinMode(12, OUTPUT);
//    while (elapsedUsecSinceLastFrameSync < usecUntilFrameSync) /* wait */ ;
//    elapsedUsecSinceLastFrameSync -= usecUntilFrameSync;
    digitalWrite(12, LOW);
    // WS2811 update begins immediately after falling edge of frame sync
//    digitalWrite(13, HIGH);
    for (int i = 0; i < leds.numPixels(); i++) {
      leds.setPixel(i, buffer[i]);
    }
    leds.show();
    digitalWrite(13, LOW);
    bytesRead = 0;
//    digitalWrite(13, LOW);
  } else {
    digitalWrite(13, HIGH);
//    Serial.println("Blah");
  }
}
