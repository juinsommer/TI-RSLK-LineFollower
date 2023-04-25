#ifndef PID_H_
#define PID_H_

#define DESIREDMAX 230                   // maximum rotations per minute
#define DESIREDMIN 30                   // minimum rotations per minute (works poorly at 30 RPM due to 16-bit timer overflow)
#define TACHBUFF 10                      // number of elements in tachometer array
#define DELTA_T 10                       // Time step in ms
#define PWMMIN 100
#define PWMMAX 7500

uint16_t LeftTach[TACHBUFF];             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection LeftDir;              // direction of left rotation (FORWARD, STOPPED, REVERSE)
int32_t LeftSteps;                       // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
uint16_t RightTach[TACHBUFF];            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection RightDir;             // direction of right rotation (FORWARD, STOPPED, REVERSE)
int32_t RightSteps;                      // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)
uint16_t Actual[2];
uint32_t UL, UR;

void getActual_Start();
uint16_t * getActual_End();

void pid(uint16_t DesiredL, uint16_t DesiredR, uint16_t *Actual);

#endif
