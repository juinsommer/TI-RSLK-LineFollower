#include "msp.h"
#include "../inc/Reflectance.h"
#include "../inc/Clock.h"
#include "../inc/Motor.h"
#include "../inc/LaunchPad.h"
#include "../inc/SysTickInts.h"
#include "../inc/CortexM.h"
#include "../inc/BumpInt.h"

void motorState(uint8_t state);
void SysTick_Handler(void);
void collision(uint8_t);

struct State {
  uint32_t out;                // 2-bit output
  const struct State *next[4]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define Center   &fsm[0]
#define Left     &fsm[1]
#define Right    &fsm[2]
#define SearchF  &fsm[3]
#define SearchB  &fsm[4]
#define SearchL  &fsm[5]
#define SearchR  &fsm[6]
#define Lost     &fsm[7]

State_t fsm[8]={
//          00      01      10     11
    {0x1, {Center, Left, Right, SearchF}},   // Center

    {0x2, {Center, Left, Right, SearchF}},   // Left
    {0x3, {Center, Left, Right, SearchF}},   // Right
    {0x4, {Center, Left, Right, SearchB}},   // SearchF
    {0x5, {Center, Left, Right, SearchL}},   // SearchB
    {0x6, {Center, Left, Right, SearchR}},   // SearchL
    {0x7, {Center, Left, Right, Lost}},      // SearchR
    {0x8, {Lost, Lost,  Lost,  Lost }}       // Lost
};

State_t *Spt;  // pointer to the current state
uint8_t Input;
uint8_t Output;
volatile uint8_t refl_data;

int main(void){
  Clock_Init48MHz();
  Motor_Init();
  Reflectance_Init();
  SysTick_Init(48000, 2);
  LaunchPad_Init();
  BumpInt_Init(&collision);

  Spt = Center;

  EnableInterrupts();

  while(1)
      WaitForInterrupt();
}

void motorState(uint8_t state) {
    switch(state){
        case 0x1:
            Motor_Forward(3000, 3000); // Center
            break;
        case 0x2:
            Motor_Left(2000, 2000); // Left
            break;
        case 0x3:
            Motor_Right(2000, 2000); // Right
            break;
        case 0x4:
            Motor_Forward(2000, 2000); // SearchF
            Clock_Delay1ms(50);
            break;
        case 0x5:
            Motor_Backward(2000, 2000); // SearchB
            Clock_Delay1ms(100);
            break;
        case 0x6:
            Motor_Left(2000, 2000); // Search Left
            Clock_Delay1ms(150);
            break;
        case 0x7:
            Motor_Right(2000, 2000); // Search right
            Clock_Delay1ms(300);
        case 0x8:
            Motor_Stop(); // Lost
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

void collision(uint8_t bump){
    switch(bump){
    case BIT0:
    case BIT2:
    case BIT3:
    case BIT5:
    case BIT6:
    case BIT7:
        Motor_Stop();
        break;
    default:
        break;
    }
}

