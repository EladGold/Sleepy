


#define  ledpin1  9  // light connected to digital pin 9
#define  ledpin2  10  // light connected to digital pin 9
#define  ledpin3  11  // light connected to digital pin 9

#define interPin 2


// scaling Variables


#define upperBoundMapped 150
#define lowerBoundMapped 10
#define upperBoundLed 1275
#define lowerBoundLed 875
#define ledChangeRatio 8 //the ammount we want every change in the sesor to be converted to light. one degree in the angle of the sensor will be translated into the number placed here
#define correction 200 // the correction needs to be applied to the sensor measurments
#define CHECKTIME 100 // the amount of sensor reads before measuring if there was a change in the direction of the sesnor slope  



// calibration variables
#define calibrationTime  33000 //time of the whole calibration
#define preCalibrationTime  18000 // time of first section of calibration

#define upperStart -100000000
#define lowerStart 100000000


//Circle parameters

#define PIN  13 // circle LEDS
#define NUM_LEDS 24


#define PIN_SECOND  7 // second circle of  LEDS
#define NUM_LEDS_SECOND 16
 

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

//#define blinkState false
