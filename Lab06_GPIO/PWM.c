#include "msp.h"

//***************************PWM_Init12*******************************
// PWM outputs on P2.4, P2.5
// Inputs:  period (1.333us)
//          duty1
//          duty2
// Outputs: none
// SMCLK = 48MHz/4 = 12 MHz, 83.33ns
// Counter counts up to TA0CCR0 and back down
// Let Timerclock period T = 8/12MHz = 666.7ns
// P2.4=1 when timer equals TA0CCR1 on way down, P2.4=0 when timer equals TA0CCR1 on way up
// P2.5=1 when timer equals TA0CCR2 on way down, P2.5=0 when timer equals TA0CCR2 on way up
// Period of P2.4 is period*1.333us, duty cycle is duty1/period
// Period of P2.5 is period*1.333us, duty cycle is duty2/period
void PWM_Init12(uint16_t period, uint16_t duty1, uint16_t duty2){
  if(duty1 >= period) return; // bad input
  if(duty2 >= period) return; // bad input
  P2->DIR |= BIT6|BIT7;          // P2.6, P2.7 output
  P2->SEL0 |= BIT6|BIT7;         // P2.6, P2.7 Timer0A functions
  P2->SEL1 &= ~(BIT6|BIT7);        // P2.6, P2.7 Timer0A functions
  TIMER_A0->CCTL[0] = 0x0080;      // CCI0 toggle
  TIMER_A0->CCR[0] = period;       // Period is 2*period*8*83.33ns is 1.333*period
  TIMER_A0->EX0 = 0x0000;        //    divide by 1
  TIMER_A0->CCTL[3] = 0x0040;      // CCR1 toggle/reset
  TIMER_A0->CCR[3] = duty1;        // CCR1 duty cycle is duty1/period
  TIMER_A0->CCTL[4] = 0x0040;      // CCR2 toggle/reset
  TIMER_A0->CCR[4] = duty2;        // CCR2 duty cycle is duty2/period
  TIMER_A0->CTL = 0x02F0;        // SMCLK=12MHz, divide by 8, up-down mode
// bit  mode
// 9-8  10    TASSEL, SMCLK=12MHz
// 7-6  11    ID, divide by 8
// 5-4  11    MC, up-down mode
// 2    0     TACLR, no clear
// 1    0     TAIE, no interrupt
// 0          TAIFG
}

//***************************PWM_Duty1*******************************
// change duty cycle of PWM output on P2.4
// Inputs:  duty1
// Outputs: none
// period of P2.6 is 2*period*666.7ns, duty cycle is duty1/period
void PWM_Duty1(uint16_t duty1){
  //if(duty1 >= TIMER_A0->CCR[0]) return; // bad input
  TIMER_A0->CCR[3] = duty1;        // CCR1 duty cycle is duty1/period
}

//***************************PWM_Duty2*******************************
// change duty cycle of PWM output on P2.5
// Inputs:  duty2
// Outputs: none// period of P2.7 is 2*period*666.7ns, duty cycle is duty2/period
void PWM_Duty2(uint16_t duty2){
  //if(duty2 >= TIMER_A0->CCR[0]) return; // bad input
  TIMER_A0->CCR[4] = duty2;        // CCR2 duty cycle is duty2/period
}
