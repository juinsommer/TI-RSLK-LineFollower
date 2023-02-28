#include <stdint.h>
#include "msp.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"

#define PERIOD 7500

// ------------Motor_Init------------
// Initialize GPIO pins for output, which will be
// used to control the direction of the motors and
// to enable or disable the drivers.
// The motors are initially stopped, the drivers
// are initially powered down, and the PWM speed
// control is uninitialized.
// Input: none
// Output: none
void Motor_Init(void){
    // P5.4 - left motor, P5.5 - right motor
    P5->SEL0 &= ~(BIT4|BIT5);
    P5->SEL1 &= ~(BIT4|BIT5);
    P5->DIR |= BIT4|BIT5;
    P5->OUT &= ~(BIT4|BIT5);

    // DRV8838 sleep pins P3.6, P3.7
    P3->SEL0 &= ~(BIT6|BIT7);
    P3->SEL1 &= ~(BIT6|BIT7);
    P3->DIR |= BIT6|BIT7;
    P3->OUT &= ~(BIT6|BIT7); // Sleep right and left motor

    // DRV8838 PWM pins P2.6, P2.7
    P2->SEL0 &= ~(BIT6|BIT7);
    P2->SEL1 &= ~(BIT6|BIT7);
    P2->DIR |= BIT6|BIT7;
    P2->OUT &= ~(BIT6|BIT7);

    PWM_Init34(PERIOD,0,0);
}

// ------------Motor_Stop------------
// Stop the motors, power down the drivers, and
// set the PWM speed control to 0% duty cycle.
// Input: none
// Output: none
void Motor_Stop(void){
  // write this as part of Lab 3
    // Stops both motors, puts driver to sleep
    // Returns right away
      //P1->OUT &= ~0xC0;
      P2->OUT &= ~0xC0;   // off
      P3->OUT &= ~0xC0;   // low current sleep mode

}

// ------------Motor_Forward------------
// Drive the robot forward by running left and
// right wheels forward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Forward(uint16_t leftDuty, uint16_t rightDuty){

        // Forward direction
        P5->OUT &= ~0xC0;
        P3->OUT |= 0xC0;
        PWM_Duty3(rightDuty);
        PWM_Duty4(leftDuty);
}

// ------------Motor_Right------------
// Turn the robot to the right by running the
// left wheel forward and the right wheel
// backward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Right(uint16_t leftDuty, uint16_t rightDuty){

    P3 -> OUT |= 0xC0;  // nSleep = 1
    P5 -> OUT &= ~0x10; // P5.4 PH = 0
    P5 -> OUT |= 0x20; // P5.5 PH = 1
    PWM_Duty4(leftDuty);
    PWM_Duty3(rightDuty);

}

// ------------Motor_Left------------
// Turn the robot to the left by running the
// left wheel backward and the right wheel
// forward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Left(uint16_t leftDuty, uint16_t rightDuty){

    P3 -> OUT |= 0xC0;  // nSleep = 1
    P5 -> OUT |= 0x10; // P5.4 PH = 1
    P5 -> OUT &= ~0x20; // P5.5 PH = 0
    PWM_Duty4(leftDuty);
    PWM_Duty3(rightDuty);

}
