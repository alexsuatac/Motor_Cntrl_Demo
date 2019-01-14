#include "MKL25Z4.h"

#define MASK(x) (1UL << (x))
#define MOTORA1 		29 //port E
#define MOTORA2 		30 //port E
#define STBY    		5 //port E
//#define CHANNELA 		4 //port D
#define GEAR_RATIO  120 //Demo motor has 120:1 gear ratio
#define PPR 				8  //Demo motor encoder outputs 8 pulses per channel per motor shaft roation 
#define TICK_QUARTERSEC 5.999999e6 // scales the PIT countdown such that the math can be done with bit shifts
#define TICK_HALFSEC 11.999999e6
#define TICK_16thSEC 1.499999e6
#define TICK_32ndSEC 7.49999e5
#define TICK_10ms    2.39999e5
#define TICK_1ms     2.3999e4
//#define RAD_32 
#define RPM_BUFF_SIZE 11

#define TPM0_CH0 1//port C
#define TPM_CLKIN0 12 //port C
#define EXTRG_IN 0 //port C


void Init_MotorDemo(void);
void Motor_Cntrl(uint8_t command);
void Init_Encoder(void);
void Init_PIT(unsigned period);
void Start_PIT(void);
void Stop_PIT(void);
void Init_TPMinputCapture(void);

// Shared variables
//extern volatile unsigned int count;
extern volatile unsigned int rpm;
extern volatile unsigned int isRpmFull;
extern unsigned int rpm_buff[RPM_BUFF_SIZE];

