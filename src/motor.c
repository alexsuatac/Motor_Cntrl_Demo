#include <MKL25Z4.H>
#include "motor.h"

void Init_Motor(struct pwm_config setup) {
		MCG->C1 |= MCG_C1_IRCLKEN_MASK; //Enables internal reference (MCGIRCLK)

		MCG->C2 |= MCG_C2_IRCS_MASK; // Uses the 4 MHz clock
	
		SIM->SCGC6 |= setup.tpm_cg_mask ; //Enable TPM clock
		SIM->SOPT2 |= SIM_SOPT2_TPMSRC(3);
		
		// Take the 4 MHz clock / 128
		setup.tpm->SC = TPM_SC_CMOD(1) | TPM_SC_PS(4);
		// Take the result from above (31250)/16383 approx 0.5 seconds
		setup.tpm->MOD = 1024;
	
	  //Setup the output of the pin
		SIM->SCGC5 |= setup.pin_cg_mask;
		setup.pin_port->PCR[setup.pin] |= PORT_PCR_MUX(setup.pin_alt);
		// configure PWM to center aligned on low-true
		setup.tpm->CONTROLS[setup.channel].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
		// * by duty cycle percent (e.g. 0.3 = 30%) starting at 0
		setup.tpm->CONTROLS[setup.channel].CnV = setup.tpm->MOD *0;
}

void ChangeDuty(struct pwm_config setup, float dutyCycle) {
	setup.tpm->CONTROLS[setup.channel].CnV = setup.tpm->MOD *dutyCycle;
}
