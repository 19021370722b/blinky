#define NRF_LOG_MODULE_NAME pilot_light
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#include "nrf_gpio.h"
#include "boards.h"
#include "app_timer.h"

#include "pilot_light.h"


#define MAX_TICK_COUNTER (16777216)     // (2^24)

// #define this to log a message when blinking the pilot light
#undef PILOT_LIGHT_LOG

#ifdef PILOT_LIGHT
APP_TIMER_DEF(my_pilot_light);

static void pilot_light_blink(void *p_context)
{
#ifdef PILOT_LIGHT_LOG
    uint32_t t = app_timer_cnt_get();
    static int last_count = 0;

    uint32_t elapsed;
    if (t > last_count) {
        elapsed = t - last_count;
    } else {
        elapsed = (MAX_TICK_COUNTER - last_count) + t;
    }

    NRF_LOG_INFO("toggle, actual ticks: %d, elapsed ticks %d", t, elapsed);


    last_count = t;
#endif

    nrf_gpio_pin_toggle(PILOT_LIGHT);
}
#endif                          // PILOT_LIGHT_BLINK


ret_code_t pilot_light_set_rate(uint32_t ms)
{
    ret_code_t err_code = NRF_SUCCESS;

#ifdef PILOT_LIGHT
    err_code = app_timer_stop(my_pilot_light);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(my_pilot_light, APP_TIMER_TICKS(ms), NULL);
    APP_ERROR_CHECK(err_code);
#endif

    return err_code;
}



ret_code_t pilot_light_init(void)
{
    ret_code_t err_code = NRF_SUCCESS;

#ifdef PILOT_LIGHT
    nrf_gpio_cfg_output(PILOT_LIGHT);
    nrf_gpio_pin_write(PILOT_LIGHT, LEDS_ACTIVE_STATE ? 0 : 1);

    err_code = app_timer_create(&my_pilot_light, APP_TIMER_MODE_REPEATED, pilot_light_blink);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(my_pilot_light, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("pilot light initialized");
#else                           //
    NRF_LOG_INFO("no pilot light defined");
#endif

    return err_code;
}
