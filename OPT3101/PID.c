#include <stdint.h>
#include <stdio.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/UART0.h"
#include "../inc/Motor.h"
#include "../inc/TimerA1.h"
#include "../inc/Tachometer.h"
#include "PID.h"

uint16_t avg(uint16_t *array, int length)
{
  int i;
  uint32_t sum = 0;

  for(i=0; i<length; i=i+1)
  {
    sum = sum + array[i];
  }
  return (sum/length);
}

void getActual_Start(){
    int i;

    for(i = 0; i < TACHBUFF; i++){
        Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);
    }
}

uint16_t * getActual_End(){
    Actual[0] = 500000/avg(LeftTach, TACHBUFF);
    Actual[1] = 500000/avg(RightTach, TACHBUFF);

    return Actual;
}

void pid(uint16_t DesiredL, uint16_t DesiredR, uint16_t *Actual){
        uint32_t Ki = 5000;
        float Kp = 5;

        int32_t Error_L;
        int32_t Error_R;

        //Calculate Error signals
        Error_L = DesiredL - Actual[0];
        Error_R = DesiredR - Actual[1];

        //PID Control Law
        UL = (UL + (Kp*Error_L) + (Ki*Error_L/1024));  // adjust left motor
        UR = (UR + (Kp*Error_R) + (Ki*Error_R/1024));  // adjust right motor

        // Clamp output to max or min duty cycles
        if(UL < PWMMIN) UL = PWMMIN;
        if(UR < PWMMIN) UR = PWMMIN;
        if(UL > PWMMAX) UL = PWMMAX;
        if(UR > PWMMAX) UR = PWMMAX;

        Motor_Forward(UL,UR);
}
