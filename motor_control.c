#include "motor_control.h"
#include <MKL25Z4.H>

#define TRUE 1
#define FALSE 0

//volatile unsigned int count = 0;
volatile unsigned int rpm = 0;
volatile unsigned int isRpmFull = FALSE;
int unsigned rpm_buff[RPM_BUFF_SIZE];
unsigned int i = 0; // indexer for rpm buffer

//Motor Control Function 
void Init_MotorDemo(void){
	// enable two GPIO pins for motor direction inputs (PTE 29 and PTE 30) and one for STBY (motor EN - Logic HIGH, PTE 5)
	
	//Enable clock to Port E
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	//Make the pins GPIO
	PORTE->PCR[MOTORA1] &= ~(PORT_PCR_MUX_MASK);
	PORTE->PCR[MOTORA1] |= PORT_PCR_MUX(1);
	PORTE->PCR[MOTORA2] &= ~(PORT_PCR_MUX_MASK);
	PORTE->PCR[MOTORA2] |= PORT_PCR_MUX(1);
	PORTE->PCR[STBY] &= ~(PORT_PCR_MUX_MASK);
	PORTE->PCR[STBY] |= PORT_PCR_MUX(1);
	
	//Configure as outputs
	PTE->PDDR |= MASK(MOTORA1) | MASK(MOTORA2) | MASK(STBY);
} 	

void Motor_Cntrl(uint8_t command){
	//clear bits
	PTE->PCOR = MASK(STBY); 
	PTE->PCOR = MASK(MOTORA1);
	PTE->PCOR = MASK(MOTORA2);
	
	switch (command){ //TODO: implement the rest of the cases from https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide/all
		case 0:
			//stop
		  PTE->PCOR = MASK(STBY);
			break;
		
		case 1:
			//Go CW
			PTE->PSOR = MASK(MOTORA1);
			PTE->PCOR = MASK(MOTORA2);
			PTE->PSOR = MASK(STBY);
			break;
		
		default:
			PTE->PCOR = MASK(STBY);
			break;
	}
}

//void Init_Encoder(void){ 
//	//Enable clock to Port D
//	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
//	
//	//Make the pins GPIO
//	PORTD->PCR[CHANNELA] &= ~(PORT_PCR_MUX_MASK);
//	PORTD->PCR[CHANNELA] |= PORT_PCR_MUX(1);
//	
//	//Set up rising edge interrupts and pulldown resistors
//	PORTD->PCR[CHANNELA] &= ~(PORT_PCR_PS_MASK);
//	PORTD->PCR[CHANNELA] |= PORT_PCR_IRQC(0x09) | PORT_PCR_PE_MASK;
//	
//	//Set up pins as input
//	PTD->PDDR &= ~MASK(CHANNELA);
//	
//	//Enable Interrupts 
//	NVIC_SetPriority(PORTD_IRQn, 128); // 0, 64, 128 or 192
//	NVIC_ClearPendingIRQ(PORTD_IRQn); 
//	NVIC_EnableIRQ(PORTD_IRQn);
//}
	
//void PORTD_IRQHandler(void) {  
//	// clear pending interrupts
//	NVIC_ClearPendingIRQ(PORTD_IRQn);
//	if ((PORTD->ISFR & MASK(CHANNELA))) {
//		count++;
//	}
//	// clear status flags 
//	PORTD->ISFR = 0xffffffff;
//	PTE->PCOR = MASK(CHANNELA);
//}

//Get RPM from Encoder
void Init_PIT(unsigned period) {
	// Enable clock to PIT module
	SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
	
	// Enable module, freeze timers in debug mode
	PIT->MCR &= ~PIT_MCR_MDIS_MASK;
	PIT->MCR |= PIT_MCR_FRZ_MASK;
	
	// Initialize PIT0 to count down from argument 
	PIT->CHANNEL[0].LDVAL = PIT_LDVAL_TSV(period);

	// No chaining
	PIT->CHANNEL[0].TCTRL &= PIT_TCTRL_CHN_MASK;
	
	// Generate interrupts
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK;

	/* Enable Interrupts */
	NVIC_SetPriority(PIT_IRQn, 0); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(PIT_IRQn); 
	NVIC_EnableIRQ(PIT_IRQn);	
}


void Start_PIT(void) {
// Enable counter
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
}

void Stop_PIT(void) {
// Disable counter
	PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK;
}

void PIT_IRQHandler() {
	//clear pending IRQ
	NVIC_ClearPendingIRQ(PIT_IRQn);
	
	// check to see which channel triggered interrupt 
	if (PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK) {
		// clear status flag for timer channel 0
		PIT->CHANNEL[0].TFLG &= PIT_TFLG_TIF_MASK;
		
		// Do ISR work
		if (i<RPM_BUFF_SIZE){
			//rpm = (TPM0->CNT);
			if(PORTC_PCR0 & 1){
				rpm_buff[i] = TPM0->CNT;
				//clear counter
				TPM0->CNT = 0;
				i++;
			}				
		}
		else {
			i=0;
			isRpmFull = TRUE;
		}			
	}  
}

  /*This function uses the TPM_CLKIN0 on PTC12 as an external clk
  that increments the TPM0 CNT register, synchronized by 4MHz MCGIRCLK. 
  Note that SOPT4 needs to be configured to use TPM_CLKIN0 for TPM0 */
void Init_TPMinputCapture(){
	//Enable internal reference (MCGIRCLK)
	MCG->C1 |= MCG_C1_IRCLKEN_MASK; 
	
	//Select 4 MHz clock
	MCG->C2 |= MCG_C2_IRCS_MASK; 
	
	//scale down here??
	
	//Enable clk for TMP0
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	
	//Configure TPM0 to use MCGIRCLK
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(3);
	
	//Configure SOPT4 for TPM_CLKIN0
	SIM->SOPT4 |= SIM_SOPT4_TPM0CLKSEL(0);
	
	//Configure PCR for input on PTC12
	//Enable PORTC clk
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	
	//configure PTC12 to TPM_CLKIN0 (alt 4)
	PORTC->PCR[TPM_CLKIN0] &= ~(PORT_PCR_MUX_MASK);
	PORTC->PCR[TPM_CLKIN0] |= PORT_PCR_MUX(4);
	//configure PTC0 to EXTRG_IN (alt 3)
	PORTC->PCR[EXTRG_IN] |= PORT_PCR_MUX(3);
	
	//Set up pins as inputs
	PTC->PDDR &= ~MASK(TPM_CLKIN0);
	PTC->PDDR &= ~MASK(EXTRG_IN);
	
	//Select EXTRG_IN as trigger for TPM0 and CSOT to start counter on trigger and reload counter to 0 when trigger is found
	//NOTE must be set before enabling TPM
	TPM0->CONF = TPM_CONF_TRGSEL(0) | TPM_CONF_CSOT(1);// | TPM_CONF_CROT(1);
	
	//TPM counts on TPM_CLKIN0 input
	TPM0->SC |= TPM_SC_CMOD(2);// | TPM_SC_TOIE(0x01); 
	
	//Debug mode
	TPM0->CONF |= TPM_CONF_DBGMODE(3);

}
