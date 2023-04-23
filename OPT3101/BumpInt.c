#include <stdint.h>
#include "msp.h"
#include "../inc/Motor.h"

void (*collision_handle)(uint8_t);

// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
// Interrupt on falling edge (on touch)
void BumpInt_Init(void(*task)(uint8_t)){
    // Store collision handle function
    collision_handle = task;

    // Bump Switches Init //
    P4->SEL0 &= ~0xED;
    P4->SEL1 &= ~0xED;   //configure P4.0, P4.2, P4.3, P4.5, P4.6, P4.7 as GPIO
    P4->DIR &= ~0xED;    //make them inputs
    P4->REN |= 0xED;     //enable pull resistors on P4.0, P4.2, P4.3, P4.5, P4.6, P4.7
    P4->OUT |= 0xED;     //make them pull-up

    // Falling Edge Interrupt Init //
    P4->IES |= 0xED;        // P4.0, P4.2, P4.3, P4.5, P4.6, P4.7 are falling edge events
    P4->IFG &= ~0xED;       // Clear interrupt flags
    P4->IE |= 0xED;         // Arm P4.0, P4.2, P4.3, P4.5, P4.6, P4.7 interrupts
    NVIC->IP[8] = (NVIC->IP[8]&0x00FFFFFF)|0x40000000;  // Priority 2
    NVIC->ISER[1] = 0x00000040;                         // Enable interrupt 38 in NVIC
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
    // write this as part of Lab 14

    uint8_t result; //declare temp variable to return
    result = ~(P4->IN)&0xED; // read the switches and store

    return result; //return
}



// we do not care about critical section/race conditions
// triggered on touch, falling edge
void PORT4_IRQHandler(void){
    // write this as part of Lab 14
    P4->IFG &= ~0xED;
    uint8_t Switch; //variables to store the read data
    Switch = Bump_Read(); //read the bump switches

    collision_handle(Switch);

//    uint32_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);
//    GPIO_clearInterruptFlag(GPIO_PORT_P4, status);
}
