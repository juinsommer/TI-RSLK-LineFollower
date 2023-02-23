#include <stdint.h>
#include "msp.h"
#include "../inc/Reflectance.h"
#include "../inc/Clock.h"
#include "../inc/MotorSimple.h"
#include "../inc/LaunchPad.h"

void motorState(uint8_t state);

struct State {
  uint32_t out;                // 2-bit output
  uint32_t delay;              // time to delay in 1ms
  const struct State *next[4]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define Center &fsm[0]
#define Left   &fsm[1]
#define Right  &fsm[2]

State_t fsm[3]={
  {0x0, 50, { Right, Left,   Right,  Center }},  // Center
  {0x2, 50, { Left,  Center, Right,  Center }},  // Left
  {0x3, 50, { Right, Left,   Center, Center }}   // Right
};

State_t *Spt;  // pointer to the current state
uint8_t Input;
uint8_t state;
uint8_t Output;

int main(void){
  Clock_Init48MHz();
  Reflectance_Init();
  Motor_InitSimple();
  LaunchPad_Init();

  Spt = Center;

  while(1){
    Output = Spt->out;            // set output from FSM
    motorState(Output);
    Clock_Delay1ms(Spt->delay);   // wait
    Input = Reflectance_Read(1000);
    state = Reflectance_Position(Input);
    Spt = Spt->next[state];       // next depends on input and state

    Clock_Delay1ms(10);
  }
}

void motorState(uint8_t state) {
    switch(state){
        case 0x3:
            Motor_LeftSimple(2000, 100);
            break;
        case 0x2:
            Motor_RightSimple(2000, 100);
            break;
        case 0x0:
            Motor_ForwardSimple(2000, 100);
            break;
        default:
            break;
    }
}
