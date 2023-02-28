#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"

// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
void Bump_Init(void){
    P4->SEL0 &= 0x12;
    P4->SEL1 &= 0x12;
    P4->DIR &= 0x12;
    P4->REN |= ~0x12;
    P4->OUT |= ~0x12;
}

// Read current state of 6 switches
// Returns a 6-bit positive logic result (0 to 63)
// bit 5 Bump5
// bit 4 Bump4
// bit 3 Bump3
// bit 2 Bump2
// bit 1 Bump1
// bit 0 Bump0
uint8_t Bump_Read(void){
    uint8_t data;

    data = ~(P4->IN)&0xFD; // Mask all 8 bits except bit 1

    return data;
}

