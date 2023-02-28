/*
 * Project1_FSM.c
 *
 *  Created on: Feb 27, 2023
 *      Author: Tamer Muddi
 */

#include <stdint.h>
#include "msp.h"
//#include "..\inc\CortexM.h"
#include "..\inc\Clock.h"
//#include "..\inc\LaunchPad.h"
//#include "..\inc\TExaS.h"
#include "..\inc\Reflectance.h"
//#include "..\inc\bump.h"

struct State {
    uint8_t Out; // Output
    uint32_t Time; // 10 ms
    const struct State *Next[12];}; // depends on 4-bit input
typedef const struct State State_t;

//define the states for the finite state machine
#define Center &FSM[0]
#define Right1 &FSM[1]
#define Left1 &FSM[2]
#define Right2 &FSM[3]
#define Left2 &FSM[4]
#define Right3 &FSM[5]
#define Left3 &FSM[6]
#define Right4 &FSM[7]
#define Left4 &FSM[8]
#define Right5 &FSM[9]
#define Left5 &FSM[10]
#define Stop &FSM[11]

State_t FSM[12]={
 {0x01, 2000,{Center,Right1,Right2,Right3,Right4,Right5,Left1,Left2,Left3,Left4}},
 {0x02, 2000,{Center,Right1,Right2,Right3,Right4,Right5,Left1,Left2,Left3,Left4}},
 {0x03, 2000,{Center,Right1,Right2,Right3,Right4,Right5,Left1,Left2,Left3,Left4}},
 {0x04, 2000,{Center,Right1,Right2,Right3,Right4,Right5,Left1,Left2,Left3,Left4}},
 {0x05, 2000,{Center,Right1,Right2,Right3,Right4,Right5,Left1,Left2,Left3,Left4}},
 {0x06, 2000,{Center,Right1,Right2,Right3,Right4,Right5,Left1,Left2,Left3,Left4}},
 {0x07, 2000,{Center,Right1,Right2,Right3,Right4,Right5,Left1,Left2,Left3,Left4}},
 {0x08, 2000,{Center,Right1,Right2,Right3,Right4,Right5,Left1,Left2,Left3,Left4}},
 {0x09, 2000,{Center,Right1,Right2,Right3,Right4,Left5,Left1,Left2,Left3,Left4}},
 {0x0A, 2000,{Center,Right1,Right2,Right3,Right4,Stop,Left1,Left2,Left3,Left4}},
 {0x0B, 2000,{Center,Right1,Right2,Right3,Right4,Stop,Left1,Left2,Left3,Left4}},
 {0x0C, 2000,{Stop,Stop,Stop,Stop,Stop,Stop,Stop,Stop,Stop,Stop}}};

int main(void){
    State_t *Pt;  // state pointer
    uint8_t temp, Input; //store the input
    Clock_Init48MHz(); // initialize clock to 48MHz

    Pt = Center; // initial state: Center

    while(1){
        //Project1_Motor(Pt->Out); //call a function that drives the motors depending on the readings
        Clock_Delay1ms(Pt->Time); // wait 1 ms * current state's Time value
        temp = Reflectance_Read(1000); //read the reflectance sensor

        switch(temp){ //convert the readings to 4 bit number
        case 00011000:
            Input = 0000;
            break;
        case 00110000:
            Input = 0001;
            break;
        case 01100000:
            Input = 0010;
            break;
        case 11000000:
            Input = 0011;
            break;
        case 10000000:
            Input = 0100;
            break;
        case 00000000:
            Input = 0101;
            break;
        case 00001100:
            Input = 0110;
            break;
        case 00000110:
            Input = 0111;
            break;
        case 00000011:
            Input = 1000;
            break;
        case 00000001:
            Input = 1001;
            break;
        }
        Pt = Pt->Next[Input]; // transition to next state

    }
}



