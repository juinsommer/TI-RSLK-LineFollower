#include "msp.h"
#include "../inc/Reflectance.h"
#include "../inc/Clock.h"
#include "../inc/Motor.h"
#include "../inc/LaunchPad.h"
#include "../inc/SysTickInts.h"
#include "../inc/CortexM.h"

void motorState(uint8_t state);
void bubbleSort(void);
void SysTick_Handler(void);
void bubbleSort(void);

struct State {
  uint32_t out;                // 2-bit output
  const struct State *next[4]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define Center   &fsm[0]
#define Left     &fsm[1]
#define Right    &fsm[2]
#define SearchF  &fsm[3]
#define SearchA  &fsm[4]
#define Lost     &fsm[5]

//#define Right1  &fsm[6]
//#define Right2  &fsm[7]
//#define Right3  &fsm[8]
//#define Right4  &fsm[9]
//#define Right5  &fsm[10]
//#define Lost    &fsm[11]

State_t fsm[6]={
//          00      01      10     11
    {0x1, {Center, Left, Right, SearchF}},   // Center

    {0x2, {Center, Left, Right, SearchF}},   // Left
    {0x3, {Center, Left, Right, SearchF}},   // Right
    {0x4, {Center, Left, Right, SearchA}},   // SearchF
    {0x5, {Center, Left, Right, SearchF}},   // SearchA
    {0x6, {Lost, Lost,  Lost,  Lost }},      // Lost
};

State_t *Spt;  // pointer to the current state
uint8_t Input;
uint8_t Output;
volatile uint8_t refl_data;

int main(void){
  Clock_Init48MHz();
  Motor_Init();
  Reflectance_Init();
  SysTick_Init(4800000,2); // 1kHz interrupts
  LaunchPad_Init();

  Spt = Center;

  EnableInterrupts();

  while(1){
      WaitForInterrupt();
  }


}

void motorState(uint8_t state) {
    switch(state){
        case 0x1:
            Motor_Forward(1500, 1500); // Center
            //Clock_Delay1ms(50);
            break;
        case 0x2:
            Motor_Left(1500, 1500); // Left
            //Clock_Delay1ms(50);
            break;
        case 0x3:
            Motor_Right(1500, 1500); // Right
            //Clock_Delay1ms(50);
            break;
        case 0x4:
            Motor_Forward(1500, 1500); // SearchF
            //Clock_Delay1ms(50);
            break;
        case 0x5:
            Motor_Left(1500, 1500);
            //Clock_Delay1ms(50);
            break;
        case 0x6:
            Motor_Stop(); // Lost
            break;
        default:
            break;
    }
}

void SysTick_Handler(void){
    volatile static uint8_t count = 0;

    if(count == 0)
        Reflectance_Start();

    else if(count == 1) {
        refl_data = Reflectance_End();
        Input = Reflectance_Position(refl_data);

        Spt = Spt->next[Input];

        Output = Spt->out;
        motorState(Output);
    }

    count++;
    if(count == 10)
        count = 0;
}

