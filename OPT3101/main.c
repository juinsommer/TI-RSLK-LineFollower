#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/I2CB1.h"
#include "../inc/CortexM.h"
#include "../inc/LPF.h"
#include "../inc/opt3101.h"
#include "../inc/LaunchPad.h"
#include "../inc/BumpInt.h"
#include "../inc/Motor.h"
#include "../inc/UART0.h"
#include "../inc/SSD1306.h"
#include "../inc/odometry.h"
#include "../inc/blinker.h"
#include "../inc/Tachometer.h"
#include "../inc/TimerA1.h"

// Prototype functions
void collision(uint8_t);
void recover();

//#define USEOLED 0
#define USEUART

#ifdef USENOKIA
// this batch configures for LCD
#include "../inc/Nokia5110.h"
#define Init Nokia5110_Init
#define Clear Nokia5110_Clear
#define SetCursor Nokia5110_SetCursor
#define OutString Nokia5110_OutString
#define OutChar Nokia5110_OutChar
#define OutUDec Nokia5110_OutUDec
#define OutSDec Nokia5110_OutSDec
#endif

#ifdef USEOLED
// this batch configures for OLED

void OLEDinit(void){SSD1306_Init(SSD1306_SWITCHCAPVCC);}
#define Init OLEDinit
#define Clear SSD1306_Clear
#define SetCursor SSD1306_SetCursor
#define OutChar SSD1306_OutChar
#define OutString SSD1306_OutString
#define OutUDec SSD1306_OutUDec
#define OutSDec SSD1306_OutSDec
#endif

#ifdef USEUART
// this batch configures for UART link to PC
#include "../inc/UART0.h"
void UartSetCur(uint8_t newX, uint8_t newY){
  if(newX == 6){
    UART0_OutString("\n\rTxChannel= ");
    UART0_OutUDec(newY-1);
    UART0_OutString(" Distance= ");
  }else{
    UART0_OutString("\n\r");
  }
}
void UartClear(void){UART0_OutString("\n\r");};
#define Init UART0_Init
#define Clear UartClear
#define SetCursor UartSetCur
#define OutString UART0_OutString
#define OutChar UART0_OutChar
#define OutUDec UART0_OutUDec
#define OutSDec UART0_OutSDec
#endif


uint32_t Distances[3];
uint32_t FilteredDistances[3];
uint32_t Amplitudes[3];
uint32_t Noises[3];
uint32_t TxChannel;
uint32_t StartTime;
uint32_t TimeToConvert; // in msec
bool pollDistanceSensor(void){
  if(OPT3101_CheckDistanceSensor()){
    TxChannel = OPT3101_GetMeasurement(Distances,Amplitudes);
    return true;
  }
  return false;
}

int32_t Mode=0; // 0 stop, 1 run
int32_t Error;
float Ki=.1;  // integral controller gain
int32_t Kp=2;  // proportional controller gain //was 4
float Kd=.01;
float UR, UL;  // PWM duty 0 to 14,998

int32_t DESIRED = 500;
int32_t SetPoint = 500;
int32_t LeftDistance,CenterDistance,RightDistance; // mm
uint32_t resultX, resultY;

#define PWMNOMINAL 5000 // was 2500
#define SWING 500 //was 1000
#define PWMMIN 1000
#define PWMMAX 5000
#define AVOIDSETPOINT 350
#define HARDAVOID 300

void Controller(void){ // runs at 100 Hz
    if(Mode){

        if((LeftDistance>DESIRED)&&(RightDistance>DESIRED)){
          SetPoint = (LeftDistance+RightDistance)/2;
        }
        else
            SetPoint = DESIRED;

        if(LeftDistance < RightDistance){
          Error = LeftDistance-SetPoint;
        }
        else{
          Error = SetPoint-RightDistance;
        }

        UR = PWMNOMINAL + Kp*Error; // proportional control
        UR = UR + Ki*Error;      // adjust right motor
        UL = PWMNOMINAL - Kp*Error; // proportional control
        UL = UL - Ki*Error;

        if(UR < (PWMNOMINAL-SWING)) UR = PWMMIN; // 3,000 to 7,000
        if(UR > (PWMNOMINAL+SWING)) UR = PWMMAX;
        if(UL < (PWMNOMINAL-SWING)) UL = PWMMIN; // 3,000 to 7,000
        if(UL > (PWMNOMINAL+SWING)) UL = PWMMAX;

        if((RightDistance < AVOIDSETPOINT) && (CenterDistance < AVOIDSETPOINT) && (LeftDistance > AVOIDSETPOINT)){
            UR = 0;
            UL = PWMMIN;
        }

        else if((LeftDistance < AVOIDSETPOINT) && (CenterDistance < AVOIDSETPOINT) && (RightDistance > AVOIDSETPOINT)){
            UL = 0;
            UR = PWMMIN;
        }

        if((RightDistance < HARDAVOID) && (CenterDistance < HARDAVOID) && (LeftDistance > HARDAVOID)){
            UR = 0;
            UL = PWMMAX;
        }

        else if((LeftDistance < HARDAVOID) && (CenterDistance < HARDAVOID) && (RightDistance > HARDAVOID)){
            UL = 0;
            UR = PWMMAX;
        }
        resultX = Odometry_GetX();
        resultY = Odometry_GetY();
        printf("Y pos: %d\n\r", resultY);

        if(resultY < 108000000)
            Motor_Forward(UL, UR);

        else{
            SoftLeftUntilTh(EAST);
            Odometry_Init(0, 0, NORTH);
        }
     }
}
extern int32_t MyX,MyY;               // position in 0.0001cm
extern int32_t MyTheta;               // direction units 2*pi/16384 radians (-pi to +pi)
extern enum RobotState Action;

void main(void){
    int i = 0;
    uint32_t channel = 1;
    DisableInterrupts();
    Clock_Init48MHz();
    Motor_Init();
    BumpInt_Init(&collision);
    LaunchPad_Init(); // built-in switches and LEDs
    Motor_Stop(); // initialize and stop
    I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
    UART0_Initprintf();

    Blinker_Init();
    Odometry_Init(0,0, NORTH);
    Action = ISSTOPPED;
    Tachometer_Init();
    TimerA1_Init(&UpdatePosition,20000); // every 40ms

    printf("\nHallway Racer\n\r");
    OPT3101_Init();
    OPT3101_Setup();
    OPT3101_CalibrateInternalCrosstalk();
    OPT3101_ArmInterrupts(&TxChannel, Distances, Amplitudes);
    TxChannel = 3;
    OPT3101_StartMeasurementChannel(channel);
    LPF_Init(100,8);
    LPF_Init2(100,8);
    LPF_Init3(100,8);
    UR = UL = PWMNOMINAL; //initial power
    EnableInterrupts();

    Mode = 1;

    while(1){
      if(TxChannel <= 2){ // 0,1,2 means new data
        if(TxChannel==0){
          if(Amplitudes[0] > 200){
            LeftDistance = FilteredDistances[0] = LPF_Calc(Distances[0]);
          }else{
            LeftDistance = FilteredDistances[0] = 1000;
          }
        }else if(TxChannel==1){
          if(Amplitudes[1] > 200){
            CenterDistance = FilteredDistances[1] = LPF_Calc(Distances[1]);
          }else{
            CenterDistance = FilteredDistances[1] = 1000;
          }
        }else {
          if(Amplitudes[2] > 200){
            RightDistance = FilteredDistances[2] = LPF_Calc2(Distances[2]);
          }else{
            RightDistance = FilteredDistances[2] = 1000;
          }
        }
//        printf("L Distance %d   R Distance %d\n\r", LeftDistance, RightDistance);
//        printf("Center Distance %d\n\r", CenterDistance);

        TxChannel = 3; // 3 means no data
        channel = (channel+1)%3;
        OPT3101_StartMeasurementChannel(channel);
        i = i + 1;
        }

        Controller();
        if(i >= 100)
          i = 0;

        WaitForInterrupt();
    }
}

void recover(){
    //Motor_Backward(3000,3000);
    Clock_Delay1ms(2000);
    Motor_Stop();
    Clock_Delay1ms(500);
}

void collision(uint8_t bump){
    switch(bump){
    case BIT0:
    case BIT2:
    case BIT3:
    case BIT5:
    case BIT6:
    case BIT7:
        Motor_Stop();
        Clock_Delay1ms(1000);
        recover();
        break;
    default:
        break;
    }
}



