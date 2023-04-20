//#include <stdint.h>
//#include "msp.h"
//#include "../inc/Clock.h"
//#include "../inc/I2CB1.h"
//#include "../inc/CortexM.h"
//#include "../inc/opt3101.h"
//#include "../inc/LaunchPad.h"
//#include "../inc/UART0.h"
//
//void UartSetCur(uint8_t newX, uint8_t newY)
//{
//  if(newX == 6){
//    UART0_OutString("\n\rTxChannel= ");
//    UART0_OutUDec(newY-1);
//    UART0_OutString(" Distance= ");
//  }else{
//    UART0_OutString("\n\r");
//  }
//}
//void UartClear(void){UART0_OutString("\n\r");};
//#define Init UART0_Init
//#define Clear UartClear
//#define SetCursor UartSetCur
//#define OutString UART0_OutString
//#define OutChar UART0_OutChar
//#define OutUDec UART0_OutUDec
//
//
//uint32_t Distances[3];
//uint32_t FilteredDistances[3];
//uint32_t Amplitudes[3];
//uint32_t TxChannel;
//uint32_t StartTime;
//uint32_t TimeToConvert; // in msec
//
//bool pollDistanceSensor(void)
//{
//  if(OPT3101_CheckDistanceSensor())
//  {
//    TxChannel = OPT3101_GetMeasurement(Distances,Amplitudes);
//    return true;
//  }
//  return false;
//}
//
//
//void main(void)
//{ // busy-wait implementation
//  uint32_t channel = 1;
//  Clock_Init48MHz();
//  SysTick->LOAD = 0x00FFFFFF;           // maximum reload value
//  SysTick->CTRL = 0x00000005;           // enable SysTick with no interrupts
//  I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
//  Init();
//  Clear();
//  OutString("OPT3101");
//  SetCursor(0, 1);
//  OutString("Left =");
//  SetCursor(0, 2);
//  OutString("Centr=");
//  SetCursor(0, 3);
//  OutString("Right=");
//  SetCursor(0, 4);
//  OutString("Busy-wait");
//  OPT3101_Init();
//  OPT3101_Setup();
//  OPT3101_CalibrateInternalCrosstalk();
//  OPT3101_StartMeasurementChannel(channel);
//  StartTime = SysTick->VAL;
//
//  while(1)
//  {
//    if(pollDistanceSensor())
//    {
//      TimeToConvert = ((StartTime-SysTick->VAL)&0x00FFFFFF)/48000; // msec
//      if(TxChannel <= 2)
//      {
//        SetCursor(6, TxChannel+1);
//        OutUDec(Distances[TxChannel]);
//      }
//      channel = (channel+1)%3;
//      OPT3101_StartMeasurementChannel(channel);
//      StartTime = SysTick->VAL;
//    }
//  }
//}
