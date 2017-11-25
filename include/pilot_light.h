


/* initialize the pilot light.   NO-OP if PILOT_LIGHT is not set */
ret_code_t pilot_light_init();

/* set the blink rate (change every N ms) */

ret_code_t pilot_light_set_rate(uint32_t ms);
