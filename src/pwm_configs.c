#include <MKL25Z4.H>
#include "pwm_configs.h"	
	
struct pwm_config get_b3(){
	struct pwm_config pin; 
	pin.tpm=TPM2;
	pin.tpm_cg_mask = SIM_SCGC6_TPM2_MASK;
	pin.pin_port = PORTB;
	pin.pin_cg_mask = SIM_SCGC5_PORTB_MASK;
	pin.pin = 3;
	pin.pin_alt = 3;
	pin.channel = 1;
	return pin;
}

struct pwm_config get_e21(){
	struct pwm_config pin; 
	pin.tpm=TPM1;
	pin.tpm_cg_mask = SIM_SCGC6_TPM1_MASK;
	pin.pin_port = PORTE;
	pin.pin_cg_mask = SIM_SCGC5_PORTE_MASK;
	pin.pin = 21;
	pin.pin_alt = 3;
	pin.channel = 1;
	return pin;
}

