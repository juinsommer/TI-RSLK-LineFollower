#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"

const int32_t w[] = {33400,23800,14300,4800,-4800,-14300,-23800,-33400};

void Reflectance_Init(void){

    // Configure P5.3 control even as output
    P5->SEL0 &= ~BIT3;
    P5->SEL1 &= ~BIT3;
    P5->DIR |= BIT3;
    P5->OUT &= ~BIT3;

    // Configure P9.2 control odd as output
    P9->SEL0 &= ~BIT2;
    P9->SEL1 &= ~BIT2;
    P9->DIR |= BIT2;
    P9->OUT &= ~BIT2;

    // Configure reflectance sensor as input
    P7->SEL0 = 0x00;
    P7->SEL1 = 0x00;
    P7->DIR  = 0x00;
    P7->OUT  = 0x00;
}

uint8_t Reflectance_Read(uint32_t time){
    uint8_t result;

    // Turn on all IR LEDs
    P5->OUT |= BIT3;
    P9->OUT |= BIT2;

    P7->DIR  = 0xFF; // Prepare to charge by setting as output
    P7->OUT  = 0xFF; // Charge capacitor

    Clock_Delay1us(10); // Wait 10 us

    P7->DIR = 0x00; // Configure as input (high impedance) to read sensors

    Clock_Delay1us(time); // Wait for time us

    // Read sensors
    int i = 0;
    for(i = 0; i < 100; i++)
        result = P7IN;

    // Turn off IR LEDs
    P5->OUT &= ~BIT3;
    P9->OUT &= ~BIT2;

    return result; // Return 8-bit sensor value
}

uint8_t Reflectance_Position(uint8_t data){
    int32_t numerator = 0, denominator = 0;
    int i = 0;

    // Iterate over 8 states and 8 data bits to calculate weighted average distance
    for(i = 0; i < 8; i++) {
        // If the i_th bit of data is not 0
        if(data&1<<i) {
            numerator += (w[i] * (data&1<<i)); // Sum the product of sensor distance and data state
            denominator += (data&1<<i); // Sum data state
        }
    }

    int32_t distance;
    distance = numerator/denominator;

    // Lost
    if(distance == 0)
        return 0x3;

    // Go forward
    if(distance > -10000 && distance < 10000)
        return 0x0;

    // Go Left
    if(distance > 10000 && distance < 20000)
        return 0x2;

    // Go right
    if(distance < -10000 && distance > -20000)
        return 0x1;

    // Go Hard Right
    if(distance > 20000)
        return 0x5;

    // Go Hard Left
    if(distance < -20000)
        return 0x4;

    else
        return 0x3;
}

void Reflectance_Start(void){
    // Turn on LEDs
    P5->OUT |= BIT3;
    P9->OUT |= BIT2;

    P7->DIR = 0xFF; // Configure as output
    P7->OUT = 0xFF; // Charge capacitors

    Clock_Delay1us(10);
    P7->DIR = 0x00; // Configure as input
}

uint8_t Reflectance_End(void){
    uint8_t result;

    result = (P7->IN&0xFF); // Read sensors

    // Turn off LEDs
    P5->OUT &= ~BIT3;
    P9->OUT &= ~BIT2;

    return result;
}

