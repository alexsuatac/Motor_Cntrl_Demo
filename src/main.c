#include <MKL25Z4.H>
#include <stdio.h>
#include "motor.h"
#include "motor_control.h"
#include "util.h"
#include "Uart.h"
#include "LCD_4bit.h"
#define TRANSFER_FCN 0

#if TRANSFER_FCN

uint16_t big_array[1000];
char serial_buff[5];

//TEST MAIN
int main (void) {
	Init_LCD();
	Clear_LCD();
	Set_Cursor(0,0);
	
	int i=0;
	
	//Enable internal reference (MCGIRCLK)
	MCG->C1 |= MCG_C1_IRCLKEN_MASK;
	
	//Select 32 kHz clock (IRC)
	MCG->C2 &= ~(MCG_C2_IRCS_MASK); 
	
	//scale down here??
	
	//Enable clk for TMP0
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	
	//Configure TPM0 to use MCGIRCLK
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(3);
	
	//Configure SOPT4 for TPM_CLKIN0
	//SIM->SOPT4 |= SIM_SOPT4_TPM0CLKSEL(0);
	
	//Configure PCR for input on PTC1 (TPM0_CH0) and PTC0 (EXTRG_IN)
	//Enable PORTC clk
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	
	//configure PTC1 to TPM0_CH0 (alt 4)
	PORTC->PCR[TPM0_CH0] &= ~(PORT_PCR_MUX_MASK);
	PORTC->PCR[TPM0_CH0] |= PORT_PCR_MUX(4);
	//configure PTC0 to EXTRG_IN (alt 3)
	PORTC->PCR[EXTRG_IN] |= PORT_PCR_MUX(3);
	
	//Set up pins as input
	PTC->PDDR &= ~MASK(TPM0_CH0);
	PTC->PDDR &= ~MASK(EXTRG_IN);

	//Select EXTRG_IN as trigger for TPM0 and CSOT to start counter on trigger and reload counter to 0 when trigger is found
	//NOTE must be set before enabling TPM
	TPM0->CONF = TPM_CONF_TRGSEL(0) | TPM_CONF_CSOT(1) | TPM_CONF_CROT(1);
	
	//TPM counts on 32kHz IRC (enable TPM)
	TPM0->SC |= TPM_SC_CMOD(1); 
	
	//Count on rising edge 
	TPM0->CONTROLS[0].CnSC |= 0x04; //0000 0100
	
	//Debug mode
	//TPM0->CONF |= TPM_CONF_DBGMODE(3);
	
	while(i < 16) {
		Clear_LCD();
		sprintf((char *)serial_buff, "%i",TPM0->CONTROLS[0].CnV);
		Print_LCD(serial_buff);
		if (TPM0->STATUS & TPM_STATUS_TOF_MASK){
			//Clear TOF w1c
			TPM0->STATUS |= TPM_STATUS_TOF(1);
		}
		if (TPM0->STATUS & TPM_STATUS_CH0F_MASK) {
			TPM0->STATUS |= TPM_STATUS_CH0F(1); // reset channel 0 flag w1c
			big_array[i] = TPM0->CONTROLS[0].CnV;
			i++;
		}
	}
	Print_LCD(" End while");
	UART0_Init(57600);

	__enable_irq();

	
	for(i = 0; i<16; i++) {
		sprintf((char *)serial_buff, "%d ",big_array[i]);
		UART0_Send((uint8_t*)serial_buff); 
		Delay(100);
		serial_buff[0] = 0;
		serial_buff[1] = 0;
		serial_buff[2] = 0;
		serial_buff[3] = 0;
		serial_buff[4] = 0;
	}

	UART0_Send((uint8_t*)"q");

	  __disable_irq();
	while(1){}
	}

#else
//Motor speed 
char serial_buff[5];
	
int main (void) {
	struct pwm_config config = get_b3();
	Init_Motor(config);
	Init_MotorDemo();
	//Init_Encoder();
	Init_TPMinputCapture();
  Init_PIT(TICK_HALFSEC); 
	
	Init_LCD();
	Clear_LCD();
	Set_Cursor(0,0);
	Print_LCD("RPM Demo Begin");
	
	float ramp = 0.0;
	int lcd_count = 0;
	char lcd_buff[40];
	
	UART0_Init(57600);
	
	__enable_irq();
	Motor_Cntrl(0);
	//ChangeDuty(config, 0.5);
	Start_PIT();
	//Delay(1000);
  
	

	
	//sprintf((char *)serial_buff, " Motor running... ");
	//UART0_Send((uint8_t*)serial_buff);
	
	while(1){
		Motor_Cntrl(2); // move CCW
		ChangeDuty(config,0.3);
		Delay(500);
		Motor_Cntrl(1);//CW
		Delay(500);
		Motor_Cntrl(0);
		ChangeDuty(config,0.0);
		
		//Clear_LCD();
		//Print_LCD("in while");
		  //ramp += 0.05;
		  //ChangeDuty(config, 0.5); // send PWM out of pin B3
			//Delay(10);
		  //snprintf((char *)lcd_buff, 40, "RPM: %d", rpm*10); //FOR DEMO
			//snprintf((char *)lcd_buff, 40, "RPM: %.3f", (float)rpm/8.0);
		//if(lcd_count == 6){
				//Clear_LCD();
				//Print_LCD(lcd_buff);
			  //Set_Cursor(0,1);
			  //snprintf((char *)lcd_buff, 40, "Duty Cycle: %.2f", ramp);
			  //Print_LCD(lcd_buff);
			//	lcd_count = 0;
			//}
			//lcd_count++;
		 	
			//Delay(100); 
	} //end while
	//Clear_LCD();
	//Print_LCD("rpmFull");
			//Send data in buffer to python script
		/*  for (int i = 0; i<RPM_BUFF_SIZE; i++){
				sprintf((char *)serial_buff, "%d ",rpm_buff[i]); //*0.75 for transformation at F = 1 sec
				UART0_Send((uint8_t*)serial_buff); 
				Delay(100);
				serial_buff[0] = 0;
				serial_buff[1] = 0;
				serial_buff[2] = 0;
				serial_buff[3] = 0;
				serial_buff[4] = 0;
		  }
	  //Send terminator "q" to python script		
		sprintf((char *)serial_buff, "q");
		UART0_Send((uint8_t*)serial_buff);*/
		
		Stop_PIT(); 
		//Motor_Cntrl(0);
		isRpmFull = 0;
	  Delay(1000);
	  __disable_irq();
	while(1){}
}


#endif