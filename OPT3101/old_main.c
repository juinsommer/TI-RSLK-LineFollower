//#include <stdint.h>
//#include <stdio.h>
//#include "msp.h"
//#include "../inc/Clock.h"
//#include "../inc/I2CB1.h"
//#include "../inc/CortexM.h"
//#include "../inc/opt3101.h"
//#include "../inc/LaunchPad.h"
//#include "../inc/UART0.h"
//#include "../inc/Motor.h"
//#include "../inc/Tachometer.h"
//#include "../inc/SysTickInts.h"
//#include "PID.h"
//
//void SysTick_Handler(void);
//
//struct State {
//  uint32_t out;                // 2-bit output
//  const struct State *next[6]; // Next if 2-bit input is 0-3
//};
//typedef const struct State State_t;
//
//#define Center   &fsm[0]
//#define Left     &fsm[1]
//#define Right    &fsm[2]
//#define SearchF  &fsm[3]
//#define SearchB  &fsm[4]
//#define SearchL  &fsm[5]
//#define SearchR  &fsm[6]
//#define Lost     &fsm[7]
//#define HLeft    &fsm[8]
//#define HRight   &fsm[9]
//
//State_t fsm[10]={
////          00      01      10     11     100     101
//    {0x1, {Center, Left, Right, SearchF, HLeft, HRight}},   // Center
//
//    {0x2, {Center, Left, Right, SearchF, HLeft, HRight }},   // Left
//    {0x3, {Center, Left, Right, SearchF, HLeft, HRight}},   // Right
//    {0x4, {Center, Left, Right, SearchB, HLeft, HRight}},   // SearchF
//    {0x5, {Center, Left, Right, SearchL, HLeft, HRight}},   // SearchB
//    {0x6, {Center, Left, Right, SearchR, HLeft, HRight}},   // SearchL
//    {0x7, {Center, Left, Right, Lost,    HLeft, HRight}},   // SearchR
//    {0x8, {Lost,   Lost, Lost,  Lost,    Lost, Lost }},     // Lost
//    {0x9, {Center, Left, Right, SearchF, HLeft, HRight }},  // HLeft
//    {0xA, {Center, Left, Right, SearchF, HLeft, HRight }}   // HRight
//};
//
//State_t *Spt;  // pointer to the current state
//uint8_t Input;
//uint8_t Output;
//volatile uint8_t refl_data;
//uint16_t *CurrentSpeed;
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
//uint32_t channel = 1;
//
//void main(void){
//    Clock_Init48MHz();
//    I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
//    Tachometer_Init();
//    Init();
//    Clear();
//    OPT3101_Init();
//    OPT3101_Setup();
//    Motor_Init();
//    SysTick_Init(480000, 1);
//    OPT3101_CalibrateInternalCrosstalk();
//    OPT3101_StartMeasurementChannel(channel);
//    UART0_Initprintf();
//    EnableInterrupts();
//
//    UL = 1875;
//    UR = 1875;
//    Spt = Center;
//
//    while(1){WaitForInterrupt();}
//
//}
//
//void SysTick_Handler(void){
//    volatile static uint8_t count = 0;
//
//    Tachometer_Get(&LeftTach[count], &LeftDir, &LeftSteps, &RightTach[count], &RightDir, &RightSteps);
//
//    if(count == 9) {
//        printf("L Actual %d\n\r", CurrentSpeed[0]);
//        printf("R Actual %d\n\r", CurrentSpeed[1]);
//
//        CurrentSpeed = getActual_End();
//
//        if(pollDistanceSensor()){
//            if(Distances[1] > 500)
//                pid(50,53, CurrentSpeed);
//
//            else
//                Motor_Stop();
//
//            channel = (channel+1)%3;
//            OPT3101_StartMeasurementChannel(channel);
//        }
//    }
//    count++;
//    if(count == 10) count = 0;
//}
