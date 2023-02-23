#include <stdint.h>
#include "msp432.h"
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
    P5OUT &= ~BIT3;
    P9OUT &= ~BIT2;

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

    int32_t distance = numerator/denominator;
    uint8_t state;

    if(numerator == 0)
        return 0x0;

    if(distance > 23800)
        state = 0x2;

    else if(distance < -23800)
        state = 0x1;

    else if(distance > 0 && distance < 4800)
        state = 0x0;

    return state;
}
