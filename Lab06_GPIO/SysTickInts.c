#include <stdint.h>
#include "msp.h"


// **************SysTick_Init*********************
// Initialize SysTick periodic interrupts
// Input: interrupt period
//           Units of period are in bus clock period
//           Maximum is 2^24-1
//           Minimum is determined by execution time of the ISR
// Input: priority 0 (high) to 7 (low)
// Output: none
void SysTick_Init(uint32_t period, uint32_t priority){
  SysTick->CTRL = 0;              // 1) disable SysTick during setup
  SysTick->LOAD = period - 1;     // 2) reload value sets period
  SysTick->VAL = 0;               // 3) any write to current clears it
  SCB->SHP[11] = priority<<5;     // set priority into top 3 bits of 8-bit register
  SysTick->CTRL = 0x00000007;     // 4) enable SysTick with core clock and interrupts
}

