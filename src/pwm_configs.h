#include "MKL25Z4.h"
struct pwm_config {
   TPM_Type * tpm;
	 uint32_t tpm_cg_mask;
	 PORT_Type * pin_port;
	 uint32_t pin_cg_mask;
	 uint32_t pin;
	 uint32_t pin_alt;
	 uint32_t channel;
};

struct pwm_config get_b3(void);
struct pwm_config get_e21(void);
