#include "additionalFunc.h"
#include <Adafruit_NeoPixel.h>
#include <CapacitiveSensor.h>
#include <stdint.h>
#include "Config.h"


// A function to determine the slope of light change
int inOutChange (int output, int prevOutput){
    if (output - prevOutput > 0){
   return 1;
  }
  else {
    return -1;
  }
}

void lightChangeLeader (double lightForce, unsigned long currentMillis, unsigned long checker,unsigned long previousMillis, int inOut, double changeRatio, Adafruit_NeoPixel &strip, int circle[],uint32_t color){

  if (lightForce >= 150){                       // start decreasing light when it reaches 150
    inOut*=-1;
    lightForce = 150;
    lightCircle(lightForce, strip, circle, color);
  }
  if (lightForce <= 10){                        // start increasing light when it reaches 10
    inOut*=-1;
    lightForce = 10;
    lightCircle(lightForce, strip, circle, color);   
  }
  if (currentMillis - checker >= 50){           // update every 50 miliseconds
    checker = updateMillis(previousMillis);                   // update cheker
    lightForce+=0.5*(inOut*changeRatio);        // update the light with half of the change ratio in the right direction (+ or -)
    lightCircle(lightForce, strip, circle, color);
  }
}


// ***This functions activates the neopixel light circle during the process according to the breathing***

void lightCircle (double lightForce, Adafruit_NeoPixel &str, int circ[],uint32_t color ){
    if (lightForce > 150){
        lightForce = 150;
    }
    if (lightForce < 0){
        lightForce = 0;
    }
    //lights the circle
    for(int i=0; i<NUM_LEDS; i++) {
        str.setBrightness(lightForce); // 40/255 brightness (about 15%)
        str.setPixelColor(circ[i], color);             // Draw 'head' pixel
    }
    str.show();
}



// A function that recieves brightness level and color. 
// sets the led circle to that brightness in that color.
void setLight (int brightness, uint32_t color, Adafruit_NeoPixel &str, int circ[]) {
    str.setBrightness(brightness);
    for(int i=0; i < NUM_LEDS; i++) {
      str.setPixelColor(circ[i], color);           
    }
    str.show();
}

// A function that recieves brightness level and color. 
// sets the led circle to that brightness in that color.
void setLight2 (uint32_t color, Adafruit_NeoPixel &str, int circ[]) {
//    strip2.setBrightness(brightness);           
    for(int i=0; i < NUM_LEDS_SECOND; i++) {
      str.setPixelColor(circ[i], color);           
    }
    str.show();
}

// A function that recieves brightness level and color. 
// sets the led circle to that brightness in that color.
void thinkingLight (uint32_t color, Adafruit_NeoPixel &str, int circ[], int i, int brightness) {
            
      int sizeOfArray = sizeof(circ)/sizeof(circ[0]); //the length of the circ array
      Serial.println (i);
      str.setBrightness(brightness); 
      str.setPixelColor(i%NUM_LEDS,0 );  
      str.setPixelColor((i+1)%NUM_LEDS, color);          
      str.show();}



// ***This functions updates the milliseconds***

unsigned long updateMillis (unsigned long previousMillis){
  return (millis()-previousMillis);
}
