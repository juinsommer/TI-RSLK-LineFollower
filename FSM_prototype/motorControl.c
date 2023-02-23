#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"

void Motor_InitSimple(void){
// Initializes the 6 GPIO lines and puts driver to sleep
// Returns right away
// initialize P5.4 and P5.5 and make them outputs

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

    // Measure output pin P1.6
    P1->SEL0 &= ~(BIT6|BIT7);
    P1->SEL1 &= ~(BIT6|BIT7);
    P1->DIR |= BIT6;
    P1->OUT &= ~BIT6;
}

void Motor_StopSimple(void){
// Stops both motors, puts driver to sleep
// Returns right away
  P2->OUT &= ~(BIT4|BIT5);   // off
  P3->OUT &= ~(BIT6|BIT7);   // low current sleep mode
}
void Motor_ForwardSimple(uint16_t duty, uint32_t time){
// Drives both motors forward at duty (100 to 9900)
// Runs for time duration (units=10ms), and then stops
// Stop the motors and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit

    uint32_t L, i;
    L = 10000 - (uint32_t)duty;
    P5->OUT &= ~(BIT4|BIT5); // Set forward direction
    P3->OUT |= BIT6|BIT7; // Wake both motors

    for(i = 0; i < time; i++){
        P2->OUT |= BIT6|BIT7;
        Clock_Delay1us((uint32_t)duty);
        P2->OUT &= ~(BIT6|BIT7);
        Clock_Delay1us(L);
    }
    Motor_StopSimple();
}
void Motor_BackwardSimple(uint16_t duty, uint32_t time){
// Drives both motors backward at duty (100 to 9900)
// Runs for time duration (units=10ms), and then stops
// Runs even if any bumper switch is active
// Returns after time*10ms

    uint32_t L, i;
    L = 10000 - (uint32_t)duty;
    P5->OUT |= BIT4|BIT5; // Set backwards direction
    P3->OUT |= BIT6|BIT7; // Wake both motors

    for(i = 0; i < time; i++){
        P2->OUT |= BIT6|BIT7;
        Clock_Delay1us((uint32_t)duty);
        P2->OUT &= ~(BIT6|BIT7);
        Clock_Delay1us(L);
    }
    Motor_StopSimple();
}

void Motor_LeftSimple(uint16_t duty, uint32_t time){
// Drives just the left motor forward at duty (100 to 9900)
// Right motor is stopped (sleeping)
// Runs for time duration (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit
    uint32_t L, i;
    L = 10000 - (uint32_t)duty;
    P3->OUT |= BIT7; // Wake left motor
    P3->OUT &= ~BIT6; // Sleep right motor
    P5->OUT &= ~BIT4; // Set forward direction

   for(i = 0; i < time; i++){
      P2->OUT |= BIT7;
      Clock_Delay1us((uint32_t)duty);
      P2->OUT &= ~BIT7;
      Clock_Delay1us(L);
   }
   Motor_StopSimple();
}


void Motor_RightSimple(uint16_t duty, uint32_t time){
// Drives just the right motor forward at duty (100 to 9900)
// Left motor is stopped (sleeping)
// Runs for time duration (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit
    uint32_t L, i;
    L = 10000 - (uint32_t)duty;

    P3->OUT |= BIT6; // Wake right motor
    P3->OUT &= ~BIT7; // Sleep left motor
    P5->OUT &= BIT4; // Set forward direction

    for(i = 0; i < time; i++){
        P2->OUT |= BIT6;
        Clock_Delay1us((uint32_t)duty);
        P2->OUT &= ~BIT6;
        Clock_Delay1us(L);
    }
    Motor_StopSimple();
}

void testPeriod(uint16_t H, uint32_t time) {
    uint32_t i, L;
    L = 10000-(uint32_t)H;

    for(i = 0; i < time; i++){
        P1->OUT |= BIT6;
        Clock_Delay1us((uint32_t)H);

        P1->OUT &= ~BIT6;
        Clock_Delay1us(L);
    }
}
