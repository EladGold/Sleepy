#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_NeoPixel.h>
#include <CapacitiveSensor.h>
#include "Config.h"
#include "additionalFunc.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#define OUTPUT_READABLE_YAWPITCHROLL

MPU6050 mpu;



// ================================================================
//                       VARIABLES DEFINING                   
// ================================================================

CapacitiveSensor   mainTouch = CapacitiveSensor(8,9); 
CapacitiveSensor   sideTouch = CapacitiveSensor(3,4); 

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRB);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(NUM_LEDS, PIN_SECOND, NEO_GRB);

int circle[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23};
int circle2[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};

////////////////////Sleepy's colours///////////////////////////
uint32_t colorBlue = strip.Color(40, 100, 255);
uint32_t colorGreen = strip.Color(000, 255, 020);
uint32_t colorTurquoise = strip.Color(000, 255, 050);
uint32_t colorGold = strip2.Color(255, 110, 020);

int output = 0;                           // output from sensor
float prevOutput = 0;                     // previous output from sensor
float prevOutputLed = 0;                  // previous output calculated from sensor sent to led
float outputLed = 0;                      // current output calculated from sensor sent to led
double lightForce = 0;                       // parameter to be sent to lamp
//Timers//
unsigned long currentMillis = 0;          // counter that counts time since sleepy is on
unsigned long previousMillis = 0;         // start time of current session
unsigned long breathCounter = 0;          // counter that makes sure every 10 seconds, new breath rate is measured
unsigned long breathLength = 0;           // used to measure the length of the last recorded breath. used in case of a recalibration.
//////////
boolean calibrated = false;
double jointVector;                       // breathing blue vector
double prev_joint = 0;
double upperBoundBreath = 0;              // the hightest point the vector reaches during the calibration
double lowerBoundBreath = 0;              // the lowest point the vector reaches during the calibration
int state = 4;
double respiratoryRate = -1;                  // speed of the user's breath
double respTemp = -1;
//boolean gotBreath = false;            // a boolean that checks whether the breath was calculated
boolean breathUpper = false;
boolean breathLower = false;

boolean sampleReset = false;            // a boolean that dictates whaether to restart sampling breath rate
int breatingCounter = 0;
int calibrationCounter = 0;
int inOut;                                      // should the light increase or decrease. start value is based on the last two measurments of the user



// ***MPU controls / Status Variables***

bool dmpReady = false;                    // set true if DMP init was successful
uint8_t mpuIntStatus;                     // holds actual interrupt status byte from MPU
uint8_t devStatus;                        // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                      // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;                       // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];                   // FIFO storage buffer

// ***Orientation / Motion Variables***

Quaternion q;                             // [w, x, y, z]        quaternion container
VectorInt16 aa;                           // [x, y, z]           accel sensor measurements
VectorInt16 aaReal;                       // [x, y, z]           gravity-free accel sensor measurements
VectorInt16 aaWorld;                      // [x, y, z]           world-frame accel sensor measurements
VectorFloat gravity;                      // [x, y, z]           gravity vector
float euler[3];                           // [psi, theta, phi]   Euler angle container
float ypr[3];                             // [yaw, pitch, roll]  yaw/pitch/roll container and gravity vector
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };




// ================================================================
//                   INTERRUPT DETECTION ROUTINE                   
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
//                          INITIAL SETUP                       
// ================================================================

void setup() {

    strip.begin();
    strip2.begin();
    strip.show();                // Initialize all pixels to 'off'
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24;               // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(115200);        // initialize serial communication
    while (!Serial);             // wait for Leonardo enumeration, others continue immediately
    Serial.println(F("Initializing I2C devices..."));      // initialize device
    mpu.initialize();
    Serial.println(F("Testing device connections..."));    // verify connection
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));              // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // making sure it worked (returns 0 if so)
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));   // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();    // get expected DMP packet size for later comparison
    } 
    else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}



// ================================================================
//                      MAIN PROGRAM LOOP                     
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;          
    switch (state){
        case 1:{
            if (state ==4){
              break;
            }
            Serial.print("You are in case #1");
            currentMillis = updateMillis(previousMillis);
            calibrated = false;
            // Make sure sensor is calibrated before start of programs

            if (!calibrated && state != 4){
                calibrate_range(currentMillis);
                calibrated = true;
                state = 2;
                break;
            }
        }            
        case 2:{
            double temp;
            unsigned long tempBreathLength;
//            respTemp = 0;                     // temporary variable that summarises the breaths taken and resets every 10 seconds 
            boolean firstRound = true; // indicator whether it is the first breath sample;
            while (state ==2){
              Serial.print("You are in case #2"); 

              
//                if (mainTouch.capacitiveSensor(30)> 50){
//                    state = 4;
//                    break;
//              }
//              if (lightForce==80){
//                breatingCounter++;
//              }
              if (!sampleReset){
                sampleReset = true;               // boolean to check when to start a new respiratory rate check window.
                if ((respTemp < 1 || respTemp > 6)&& respTemp!=-1){
                  quickCalibrate(currentMillis, breathLength, inOut, lightForce);
//                  calibrationCounter++;
                }

                else if (!firstRound){
                  respiratoryRate = respiratoryRate*0.3 + respTemp*0.7;
                }
                else{
                  respiratoryRate = respTemp;     // respiratory rate - updated every 10 seconds     
                  firstRound = false;
                }
                currentMillis = updateMillis(previousMillis);
                breathCounter = currentMillis;    // used to count the time between updates
                //respiratoryRate = respTemp;   // respiratory rate - updated every 10 seconds     
                respTemp = 0;                     // temporary variable that summarises the breaths taken and resets every 10 seconds 
              }      
              wait();
              getData ();
              Serial.print(jointVector);
              temp = jointVector;                 // multiply readings by 100 in order to read small movements
              if (temp > upperBoundBreath){       // if the read exceeds the range, change it to the maximum value
                  temp = upperBoundBreath;
              }
              if (temp < lowerBoundBreath){       //if the read is lower then the range, change it to the minimum value
                  temp = lowerBoundBreath;
              }
              //output = map(temp ,lowerBoundBreath,upperBoundBreath,lowerBoundMapped,upperBoundMapped);
              output = map(temp ,upperBoundBreath,lowerBoundBreath,upperBoundMapped,lowerBoundMapped);
              lightForce = output;

              
               inOut = inOutChange (output, prevOutput);
 
              
              prevOutput = output;
              lightCircle (lightForce, strip, circle, colorBlue);
              
              Serial.print("lightForce: ");
              Serial.print(lightForce);
              Serial.print("\t");
              Serial.print("Respiratory Rate: ");
              Serial.print(respiratoryRate);
              Serial.print("respTemp: ");
              Serial.print(respTemp);
              Serial.print("\t");
              Serial.print("BreathLength: ");
              Serial.print(breathLength);
              Serial.print("\t");
              Serial.println();

              
              // if the temp joint vector value is higher then half the calibration range and we have not recorded a new breath yet.
              if (temp > (upperBoundBreath-((upperBoundBreath-lowerBoundBreath)*0.35)) && (!breathUpper)){  
                respTemp+=0.5;         //add half a breath to counter
                tempBreathLength = updateMillis(previousMillis);
//                gotBreath = true;   // mark this breath as measured
                  breathUpper = true;
                  breathLower = false;
              }
  
              // if the temp joint vector value is lower then half the calibration range and we have already recorded a breath 
              // (which means the user finished the previous breath)
              if (temp <= (upperBoundBreath-((upperBoundBreath-lowerBoundBreath)*0.65)) && !breathLower){
                respTemp+=0.5;       //add half a breath to counter
                breathLength = updateMillis(previousMillis) - tempBreathLength;
//                gotBreath = false;   // open the window to detect a new breath
                  breathUpper = false;
                  breathLower = true;
              }
  
              // if the sample window of 10 seconds has passed, close it, and start again
              if (currentMillis - breathCounter > 10000){ 
                sampleReset = false;
              }
              currentMillis = updateMillis(previousMillis);
  
              if (currentMillis >= 100000 + (10000*calibrationCounter) ){
//                respiratoryRate = (breatingCounter/2);
                state = 3;
                if (respiratoryRate < 1){
                  respiratoryRate = 1;
                }
                if (respiratoryRate >7){
                  respiratoryRate = 1;
                }
                break;
              }
            }
        }
        case 3:{
//            wait();
//            getData ();
            Serial.print("Respiratory Rate: ");
            Serial.print(respiratoryRate);
            Serial.print("\t");
            Serial.println();
            Serial.print("You are in case #3 "); 
            unsigned long startTime = currentMillis;        // a variable that saves the time of the beginning of the calibration
            unsigned long checker = currentMillis;          // a variable that times changes in light updates
            int baseline = respiratoryRate*6;               // The start respiratory rate of the user
//            int baseline = respiratoryRate;               // The start respiratory rate of the user
//            double timeActuallyTested = 60.0 - (10*calibrationCounter);
//            double cycleTime = (timeActuallyTested / baseline);               // the amount of time it takes the user to complete a breath (from no light to full light, and back)
            double cycleTime = 60/baseline;
            double changeRatio = 28/(cycleTime);        // how much should the light change every millisecond
            Serial.print("baseline: ");
            Serial.print(baseline);
            Serial.print("\t");
            Serial.print("cycleTime: ");
            Serial.print(cycleTime);
            Serial.print("\t");
            Serial.print("changeRatio: ");
            Serial.print(changeRatio);
            Serial.print("\t");
            inOut = inOutChange (output, prevOutput);


            unsigned long interval = 30000;                 // update breathe speed - done every 30 seconds
            //cycleTime*=1.1;                           // slow breathing by 10%
            //changeRatio = 280/(cycleTime*10);         // update changeRtaio
            while (currentMillis - startTime < 1005000 && state != 4){     // 14 minutes of cycle time
//              if (mainTouch.capacitiveSensor(30)> 100){
//                state = 4;
//                break;
//              }
              if (lightForce >= 150){                       // start decreasing light when it reaches 150
                inOut*=-1;
                lightForce = 150;
                lightCircle(lightForce, strip, circle, colorBlue);
              }
              if (lightForce <= 10){                        // start increasing light when it reaches 10
                inOut*=-1;
                lightForce = 10;
                lightCircle(lightForce, strip, circle, colorBlue);   
              }
              if (currentMillis - checker >= 50){           // update every 50 miliseconds
                checker = updateMillis(previousMillis);                   // update cheker
                lightForce+=0.5*(inOut*changeRatio);        // update the light with half of the change ratio in the right direction (+ or -)
                lightCircle(lightForce, strip, circle, colorBlue);
              }
                Serial.print("baseline: ");
                Serial.print(baseline);
                Serial.print("\t");
                Serial.print("cycleTime: ");
                Serial.print(cycleTime);
                Serial.print("\t");
                Serial.print("changeRatio: ");
                Serial.print(changeRatio);
                Serial.print("\t");
                Serial.print("inOut: ");
                Serial.print(inOut);
                Serial.print("\t");
                Serial.print("current Rate: ");
//                Serial.print(timeActuallyTested/cycleTime);
//                Serial.print("\t");
                Serial.print("ledpin: ");
                Serial.println(lightForce);
         

              // if more then the interval has passed (now set on 30 seconds) and we haven't reached 6 breaths per minute, update breathing speed
              if ((currentMillis - startTime) >= interval && cycleTime <10){    
                interval+= 60000;                         //update to recieve next update time
                Serial.print (cycleTime);
                Serial.print("\t");
                Serial.println (interval);
                cycleTime*=1.07;                           // slow breathing by 10%
                changeRatio = 280/(cycleTime*10);         // update changeRtaio
              }
              currentMillis = updateMillis(previousMillis);
            }
            state = 4;
        }
          case 4:{
            Serial.println();
            Serial.print("You are in case default "); 
            wait();
            getData ();     
            lightCircle (0, strip, circle, colorBlue);
            int power = 255;
            currentMillis = updateMillis(previousMillis);
            unsigned long timer = currentMillis;
            while (currentMillis - timer < 2000){
              currentMillis = updateMillis(previousMillis);
            }
            while (state == 4){
              currentMillis = updateMillis(previousMillis);
              if (sideTouch.capacitiveSensor(30)> 90){
                timer = currentMillis;
                Serial.println (sideTouch.capacitiveSensor(30));
                while (currentMillis - timer < 500){
                  currentMillis = updateMillis(previousMillis);
                }
                setLight2 (colorGold, strip2, circle2);
                if (power == 255){
                  power = 1;
                }
                else {
                  power = 255;
                  setLight2 (0, strip2, circle2);
                }
                while (currentMillis - timer < 1500){
                  currentMillis = updateMillis(previousMillis);
                }
              }
              Serial.print("Main Touch: ");
              Serial.print("\t");
              Serial.print (sideTouch.capacitiveSensor(30));
              Serial.print("\t");
              Serial.print("Main Touch: ");
              Serial.println (mainTouch.capacitiveSensor(30));
              if ((mainTouch.capacitiveSensor(30 )> 90 && power == 255) || (power ==1 && mainTouch.capacitiveSensor(30) > 90)){
                setLight2 (0, strip2, circle2);
                Serial.println (mainTouch.capacitiveSensor(30));
                previousMillis = updateMillis(previousMillis);
                currentMillis = updateMillis(currentMillis); 
                Serial.println (currentMillis);
                state = 1;
                break;
              }
            }
          }                 
    }
}



// ================================================================
//                      ADDITIONAL FUNCTIONS                     
// ================================================================


// ***This functions sets the breathing upper and lower bounds during the calibration***

void calibrate_range (unsigned long currentMillis){
    double upperBound = upperStart;
    double lowerBound = lowerStart;
    int brightness = 120;                                   // starting brightness for the LED circle in the calibration
    unsigned long counter = 125;                                      // avaribale that counts the difference in seconds between each time the brightness changes
    unsigned long counter2 = 250;                                      // avaribale that counts the difference in seconds between each time the brightness changes
    unsigned long timeInCalibration = currentMillis;        // timer that times the amount of time in the calibration.
    int i = 0; //index for light strip
    while (currentMillis < preCalibrationTime && state != 4) {
//        if (mainTouch.capacitiveSensor(30)> 100){
//          state = 4;
//          break;
//        }
        currentMillis = updateMillis(previousMillis);
        if (currentMillis - timeInCalibration >= counter2){  // if "counter" amount of seconds has passed since the beginning of the calibration, change brightness
//             setLight (brightness, colorGold, strip, circle);
             thinkingLight (colorGold, strip, circle, i, brightness);
             counter2+=250;                                 // update counter so the next change in the brightness will be in 1 second. 
             i+=1;
        }
//        strip.setPixelColor(circle[0],colorGold );
//        strip.setBrightness(brightness); 
//        strip.show(); 
    wait();
    getData();
    Serial.print ("jointVector:");
    Serial.println (jointVector);
    }
    while (currentMillis < calibrationTime && state != 4){

//        if (mainTouch.capacitiveSensor(30)> 100){
//          state = 4;
//          break;
//        }
        currentMillis = updateMillis(previousMillis);
        if (currentMillis - (timeInCalibration+preCalibrationTime) >= counter){
             setLight (brightness, colorTurquoise, strip, circle);
             counter+=125;
             brightness -=1;
//             Serial.println (counter);
        }
        wait();
        getData();
        Serial.print ("jointVector: ");
        Serial.print (jointVector);
        Serial.print("\t");
        Serial.print ("upper bound: ");
        Serial.print (upperBound);
        Serial.print("\t");
        Serial.print ("lower bound: ");
        Serial.println (lowerBound);
        
        if (jointVector > upperBound){
            upperBound = jointVector;
        }
        if (jointVector < lowerBound ){
            lowerBound = jointVector;
        }
    }
    upperBoundBreath = upperBound;
    lowerBoundBreath = lowerBound;
    return;
}

void quickCalibrate (unsigned long currentMillis, unsigned long breathLength, int inOut,int  lightForce){
    double upperBound = upperStart;
    double lowerBound = lowerStart;
    if (breathLength < 4000){
      breathLength = 4000;
    }
    unsigned long startTime = currentMillis;        // a variable that saves the time of the beginning of the calibration
    unsigned long checker2 = currentMillis;          // a variable that times changes in light updates
    double interval = 28000/(breathLength);

//    setLight (75, colorGreen);                       // set the color to green
    respTemp = -1;
    while (currentMillis - startTime < 10000){
      if (lightForce >= 150){                       // start decreasing light when it reaches 150
        inOut*=-1;
        lightForce = 150;
        lightCircle(lightForce, strip, circle, colorBlue);
      }
      if (lightForce <= 10){                        // start increasing light when it reaches 10
        inOut*=-1;
        lightForce = 10;
        lightCircle(lightForce, strip, circle, colorBlue);   
      }
      if (currentMillis - checker2 >= 50){           // update every 50 miliseconds
        checker2 = updateMillis(previousMillis);                   // update cheker
        lightForce+=0.5*(inOut*interval);        // update the light with half of the change ratio in the right direction (+ or -)
        lightCircle(lightForce, strip, circle, colorBlue);
      }
      wait();
      getData();
      Serial.print ("jointVector: ");
      Serial.print (jointVector);
      Serial.print("\t");
      Serial.print ("upper bound: ");
      Serial.print (upperBound);
      Serial.print("\t");
      Serial.print ("lower bound: ");
      Serial.println (lowerBound);

      if (jointVector > upperBound){
          upperBound = jointVector;
      }
      if (jointVector < lowerBound ){
          lowerBound = jointVector;
      }
      currentMillis = updateMillis(previousMillis);
   }
    upperBoundBreath = upperBound;
    lowerBoundBreath = lowerBound;
    return;
}




// ***This functions reads data from the sensor***

void getData (){
    fifoCount = mpu.getFIFOCount();                     // get current FIFO count
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {   // check for overflow (will occur only if code is inefficient)
        mpu.resetFIFO();                                // reset so we can continue cleanly
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {                   // otherwise, check for DMP data ready interrupt
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);       // read a packet from FIFO
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        prev_joint = jointVector;
        double ypr0Pow = pow((ypr[0] * 1800000/M_PI),2);
        double ypr1Pow = pow((ypr[1] * 1800000/M_PI),2);
        double ypr2Pow = pow((ypr[2] * 1800000/M_PI),2);
        jointVector = sqrt(ypr1Pow+ypr2Pow);
        jointVector = 0.3*prev_joint + 0.7*jointVector * (-1);
        mpu.resetFIFO(); // Clear the buffer in order to prevent overFlow
        return;
    }
}


// ***This functions waits for interrupt in order to make sure that enough data was read***

void wait (){
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    return;
}

/*
// ***This functions updates the milliseconds***

unsigned long updateMillis (){
  return (millis()-previousMillis);
}
*/
/*
// ***This functions activates the neopixel light circle during the process according to the breathing***

void lightCircle (double lightforce){
    if (lightForce > 150){
        lightForce = 150;
    }
    if (lightForce < 0){
        lightForce = 0;
    }
    //lights the circle
    for(int i=0; i<NUM_LEDS; i++) {
        strip.setBrightness(lightforce); // 40/255 brightness (about 15%)
        strip.setPixelColor(circle[i], colorBlue);             // Draw 'head' pixel
    }
    strip.show();
}



// A function that recieves brightness level and color. 
// sets the led circle to that brightness in that color.
void setLight (int brightness, uint32_t color) {
    strip.setBrightness(brightness);
    for(int i=0; i < NUM_LEDS; i++) {
      strip.setPixelColor(circle[i], color);           
    }
    strip.show();
}

// A function that recieves brightness level and color. 
// sets the led circle to that brightness in that color.
void setLight2 (uint32_t color) {
//    strip2.setBrightness(brightness);           
    for(int i=0; i < NUM_LEDS_SECOND; i++) {
      strip2.setPixelColor(circle2[i], color);           
    }
    strip2.show();
}
*/
