// CortexM.c
// Cortex M registers and basic functions used in these labs
// Daniel and Jonathan Valvano
// September 20, 2016
/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
 */
#include <stdint.h>
#include "../inc/Motor.h"
#include "stdio.h"
#include "string.h"
#include "../inc/UART0.h"
#include "msp.h"

//*********** DisableInterrupts ***************
// disable interrupts
// inputs:  none
// outputs: none
void DisableInterrupts(void){
  __asm ("    CPSID  I\n"
         "    BX     LR\n");
}

//*********** EnableInterrupts ***************
// enable interrupts
// inputs:  none
// outputs: none
void EnableInterrupts(void){
  __asm  ("    CPSIE  I\n"
          "    BX     LR\n");
}
//*********** StartCritical ************************
// make a copy of previous I bit, disable interrupts
// inputs:  none
// outputs: previous I bit
void StartCritical(void){
  __asm  ("    MRS    R0, PRIMASK   ; save old status \n"
          "    CPSID  I             ; mask all (except faults)\n"
          "    BX     LR\n");
}

//*********** EndCritical ************************
// using the copy of previous I bit, restore I bit to previous value
// inputs:  previous I bit
// outputs: none
void EndCritical(void){
  __asm  ("    MSR    PRIMASK, R0\n"
          "    BX     LR\n");
}

//*********** WaitForInterrupt ************************
// go to low power mode while waiting for the next interrupt
// inputs:  none
// outputs: none
void WaitForInterrupt(void){
    int stopReading = 0;
    char command;
    UART0_Init();
    while(stopReading == 0){
        if((EUSCI_A0->IFG&0x01) == 1){
            command = UART0_InChar(); //calling UART0_InChar function
            if (command == 'S') //if the input via Bluetooth is "S"
            {
              Motor_Stop(); //Stop
              stopReading = 1;
              main();
            }
        }
        else
        {
            //__asm  ("    WFI\n"
                    //"    BX     LR\n");
            continue;
        }



    }
}
	

