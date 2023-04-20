#include <stdint.h>
#include "../inc/Clock.h"
#include "../inc/TA3InputCapture.h"
#include "msp.h"
#include "../inc/Tachometer.h"

uint16_t Tachometer_FirstRightTime, Tachometer_SecondRightTime;
uint16_t Tachometer_FirstLeftTime, Tachometer_SecondLeftTime;
int Tachometer_RightSteps = 0;     // incremented with every step forward; decremented with every step backward
int Tachometer_LeftSteps = 0;      // incremented with every step forward; decremented with every step backward
enum TachDirection Tachometer_RightDir = STOPPED;
enum TachDirection Tachometer_LeftDir = STOPPED;

void tachometerRightInt(uint16_t currenttime){
  Tachometer_FirstRightTime = Tachometer_SecondRightTime;
  Tachometer_SecondRightTime = currenttime;
  if((P5->IN&0x01) == 0){
    // Encoder B is low, so this is a step backward
    Tachometer_RightSteps = Tachometer_RightSteps - 1;
    Tachometer_RightDir = REVERSE;
  }else{
    // Encoder B is high, so this is a step forward
    Tachometer_RightSteps = Tachometer_RightSteps + 1;
    Tachometer_RightDir = FORWARD;
  }
}

void tachometerLeftInt(uint16_t currenttime){
  Tachometer_FirstLeftTime = Tachometer_SecondLeftTime;
  Tachometer_SecondLeftTime = currenttime;
  if((P5->IN&0x04) == 0){
    // Encoder B is low, so this is a step backward
    Tachometer_LeftSteps = Tachometer_LeftSteps - 1;
    Tachometer_LeftDir = REVERSE;
  }else{
    // Encoder B is high, so this is a step backward
    Tachometer_LeftSteps = Tachometer_LeftSteps + 1;
    Tachometer_LeftDir = FORWARD;
  }
}

// ------------Tachometer_Init------------
// Initialize GPIO pins for input, which will be
// used to determine the direction of rotation.
// Initialize the input capture interface, which
// will be used to measure the speed of rotation.
// Input: none
// Output: none
void Tachometer_Init(void){
  // initialize P5.0 and P5.2 and make them GPIO inputs
  P5->SEL0 &= ~0x05;
  P5->SEL1 &= ~0x05;               // configure P5.0 and P5.2 as GPIO
  P5->DIR &= ~0x05;                // make P5.0 and P5.2 in
  TimerA3Capture_Init01(&tachometerRightInt, &tachometerLeftInt);
}

// ------------Tachometer_Get------------
// Get the most recent tachometer measurements.
// Input: leftTach   is pointer to store last measured tachometer period of left wheel (units of 0.083 usec)
//        leftDir    is pointer to store enumerated direction of last movement of left wheel
//        leftSteps  is pointer to store total number of forward steps measured for left wheel (360 steps per ~220 mm circumference)
//        rightTach  is pointer to store last measured tachometer period of right wheel (units of 0.083 usec)
//        rightDir   is pointer to store enumerated direction of last movement of right wheel
//        rightSteps is pointer to store total number of forward steps measured for right wheel (360 steps per ~220 mm circumference)
// Output: none
// Assumes: Tachometer_Init() has been called
// Assumes: Clock_Init48MHz() has been called
void Tachometer_Get(uint16_t *leftTach, enum TachDirection *leftDir, int32_t *leftSteps,
                    uint16_t *rightTach, enum TachDirection *rightDir, int32_t *rightSteps){
  *leftTach = (Tachometer_SecondLeftTime - Tachometer_FirstLeftTime);
  *leftDir = Tachometer_LeftDir;
  *leftSteps = Tachometer_LeftSteps;
  *rightTach = (Tachometer_SecondRightTime - Tachometer_FirstRightTime);
  *rightDir = Tachometer_RightDir;
  *rightSteps = Tachometer_RightSteps;
}

