#include <stdint.h>
#include <Adafruit_NeoPixel.h>
#include <CapacitiveSensor.h>


// A function to determine the slope of light change
int inOutChange (int output, int prevOutput);

void lightChangeLeader (double lightForce, unsigned long currentMillis, unsigned long checker ,unsigned long previousMillis, int inOut, double changeRatio, Adafruit_NeoPixel &strip, int circle[],uint32_t color);

// ***This functions activates the neopixel light circle during the process according to the breathing***
void lightCircle (double lightForce, Adafruit_NeoPixel &str, int circ[],uint32_t color);



// A function that recieves brightness level and color. 
// sets the led circle to that brightness in that color.
void setLight(int brightness, uint32_t color, Adafruit_NeoPixel &str, int circ[]);


// A function that recieves brightness level and color. 
// sets the led circle to that brightness in that color.
void setLight2 (uint32_t color, Adafruit_NeoPixel &str, int circ[]);

// ***This functions updates the milliseconds***
unsigned long updateMillis (unsigned long previousMillis);

void thinkingLight (uint32_t color, Adafruit_NeoPixel &str, int circ[], int i, int brightness);
