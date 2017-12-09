

#include "nrf_drv_saadc.h"



/* initialize the battery level measurement. */
ret_code_t control_adc_init();

/* set the battery check period (check every N ms) */
ret_code_t control_adc_set_rate(uint32_t ms);
