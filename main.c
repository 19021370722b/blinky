/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example
 * @brief Blinky Example Application main file.
 *
 * This file contains the source code for a sample application to blink LEDs.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"

#include "nrf_drv_clock.h"
#include "app_timer.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_sdh.h"


typedef enum { CW, CCW, STOP} Direction;

int leds[] = { 0, 1, 3, 2 };  // ordered in "circle" fashion from board



APP_TIMER_DEF(my_pilot_light);

void blink_pilot_light(void *p_context)
{
    nrf_gpio_pin_toggle (ARDUINO_0_PIN);
    NRF_LOG_INFO("pilot light");
}




/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


static void timers_init(void)
{
  ret_code_t err_code;

  err_code = nrf_drv_clock_init();
  APP_ERROR_CHECK(err_code);

  nrf_drv_clock_lfclk_request(NULL);

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.

    err_code = app_timer_create(&my_pilot_light, APP_TIMER_MODE_REPEATED, blink_pilot_light);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(my_pilot_light, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("timers initialized");
}



/**
 * @brief Function for application main entry.
 */
int main(void)
{
    log_init();
    NRF_LOG_INFO("start of blinky app");

    if (nrf_sdh_is_enabled()) {
      NRF_LOG_INFO("SoftDevice is enabled");
    }
    else {
      NRF_LOG_INFO("SoftDevice is NOT enabled");
    }


    /* Configure board. */
    bsp_board_leds_init();
    for (int i = 0; i < LEDS_NUMBER; i++) {
	bsp_board_led_off(i);
    }

    nrf_gpio_cfg_output(ARDUINO_0_PIN);
    nrf_gpio_pin_write(ARDUINO_0_PIN, LEDS_ACTIVE_STATE ? 1 : 0);

    timers_init();

    /* Toggle LEDs. */


    Direction current_direction = CW;

    int i = 0;
    if (current_direction == CCW) {
      i = (LEDS_NUMBER-1);
    }

    int count = 0;

    while (true)
    {
        NRF_LOG_PROCESS();

	NRF_LOG_INFO("about to invert %d, tick %d", leds[i], ++count);
	bsp_board_led_invert(leds[i]);

        switch (current_direction) {
            case CW:
              i += 1;
              break;
            case CCW:
              i -= 1;
              break;
            case STOP:
              // no change in position, just flash the LED
            default:
              break;
        }

	if (i < 0) {
	    i = (LEDS_NUMBER-1);
	}
	if (i >= LEDS_NUMBER) {
	    i = 0;
	}

	nrf_delay_ms(500);
    }
}

/**
 *@}
 **/
