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

#include "nrf_drv_gpiote.h"

#include "boards.h"

#include "nrf_drv_clock.h"
#include "app_timer.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_sdh.h"

#include "pilot_light.h"

APP_TIMER_DEF(circle_timer);

typedef enum { CW, CCW, STOP } Direction;

volatile Direction current_direction = CW;
volatile Direction previous_direction;

volatile int speed = 0;

int ticks_for_speed()
{
    if (speed >= 0 && speed <= 9) {
        return 250 * (speed + 1);
    }
    else {
        return 1000;
    }
}

void bump_speed()
{
    speed += 1;
    if (speed > 9) {
        speed = 0;
    }

    ret_code_t err_code;

    err_code = app_timer_stop(circle_timer);
    APP_ERROR_CHECK(err_code);

    int new_time = ticks_for_speed();

    err_code = app_timer_start(circle_timer, APP_TIMER_TICKS(new_time), NULL);
    APP_ERROR_CHECK(err_code);

    err_code = pilot_light_set_rate(new_time);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("speed changed to %d (%d)", new_time, speed);
}

int unused_leds[] = { };

#define NUM_UNUSED_LEDS (sizeof(unused_leds)/sizeof(unused_leds[0]))

void turn_off_leds()
{
    for (int i = 0; i < NUM_UNUSED_LEDS; i++) {
        nrf_gpio_cfg_output(unused_leds[i]);
        nrf_gpio_pin_write(unused_leds[i], LEDS_ACTIVE_STATE ? 1 : 0);
    }
}

static int next_led = 0;
void update_circle(void *p_context)
{

    static int next_led= 0;
#if 0
    static int count = 0;
    NRF_LOG_INFO("about to invert i=%d, %d, tick %d", next_led, bsp_board_led_idx_to_pin(next_led), ++count);
#endif
    bsp_board_led_invert(next_led);

    switch (current_direction) {
    case CW:
        next_led+= 1;
        break;
    case CCW:
        next_led-= 1;
        break;
    case STOP:
        // no change in position, just flash the LED
    default:
        break;
    }

    if (next_led< 0) {
        next_led= (LEDS_NUMBER - 1);
    }
    if (next_led>= LEDS_NUMBER) {
        next_led= 0;
    }
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

    err_code = app_timer_create(&circle_timer, APP_TIMER_MODE_REPEATED, update_circle);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(circle_timer, APP_TIMER_TICKS(250), NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("timers initialized");
}

bool button_pressed[4] = { false, false, false, false };

uint32_t time_delta(uint32_t start, uint32_t end)
{
    uint32_t delta;

    if (start <= end) {
	delta = end - start;
    }
    else {
	delta = (UINT32_MAX - start) + end;
    }

    return delta;
}





static void pin_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    static int last_button_press_time = 0;
    static int last_button_pressed = -1;
    uint32_t idx = -1;


    uint32_t t = app_timer_cnt_get();

    if (pin == last_button_pressed) {
	uint32_t delta = time_delta(last_button_press_time, t);
	if (delta < APP_TIMER_TICKS(50)) {
	    // NRF_LOG_INFO("button bounce %d: %d", pin, delta);
	    return;
	}
    }
    else {
	last_button_pressed = pin;
    }

    NRF_LOG_INFO("button press %d (since last button: %d)", t, time_delta(last_button_press_time,t));
    last_button_press_time = t;

    idx = bsp_board_pin_to_button_idx(pin);

    if (idx != 0xFFFFFFFF) {
	//button_pressed[idx] = !button_pressed[idx]
	int set = nrf_gpio_pin_read(pin) == BUTTONS_ACTIVE_STATE;

	if (set == button_pressed[idx]) {
	    NRF_LOG_INFO("***** REPEAT SETTING *****");
	}

	button_pressed[idx] = set;
        NRF_LOG_INFO("event button pin:%d set:%d idx:%d: %d => %s", pin, set, idx, button_pressed[idx], button_pressed[idx] ? "PRESSED" : "RELEASED");

	if (button_pressed[0] && button_pressed[1]) {
	    for (int i = 0; i < LEDS_NUMBER; i++) {
		bsp_board_led_off(i);
		next_led = (current_direction == CW) ? 0 : LEDS_NUMBER;
	    }
	}
        else if (idx == 0) {
            if (button_pressed[0]) {
                previous_direction = current_direction;
                current_direction = STOP;
            }
            else {
                current_direction = (previous_direction == CW) ? CCW : CW;
            }
        }
        else if (idx == 1 && button_pressed[1]) {
            bump_speed();
        }



    }
}



static void init_buttons()
{
    ret_code_t err_code;

    if (!nrf_drv_gpiote_is_init()) {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);

	bsp_board_buttons_init();

	nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
	config.pull = NRF_GPIO_PIN_PULLUP;

	for (int i = 0; i < BUTTONS_NUMBER; i++) {
	    int pin = bsp_board_button_idx_to_pin(i);

	    button_pressed[i] = bsp_board_button_state_get(i);

	    err_code = nrf_drv_gpiote_in_init(pin, &config, pin_event_handler);
	    APP_ERROR_CHECK(err_code);
	    nrf_drv_gpiote_in_event_enable(pin, true);
	}

        NRF_LOG_INFO("nrf_drv_gpiote_init done");

	for (int i=0; i < BUTTONS_NUMBER; i++) {
	    NRF_LOG_INFO("initial state button idx:%d pin:%d %d", i, bsp_board_button_idx_to_pin(i), button_pressed[i]);
	}

    }
    else {
        NRF_LOG_INFO("nrf_drv_gpiote_init already initialized");
    }
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    log_init();
    NRF_LOG_INFO("start of blinky app");
#ifdef TARGET_TEXT
    NRF_LOG_INFO("running on %s", TARGET_TEXT);
#endif

    /* Configure board. */
    bsp_board_leds_init();

    turn_off_leds();

    init_buttons();

    timers_init();

    pilot_light_init();
    pilot_light_set_rate(ticks_for_speed());

    /* Toggle LEDs. */

    if (nrf_sdh_is_enabled()) {
        NRF_LOG_INFO("SoftDevice is enabled");
    }
    else {
        NRF_LOG_INFO("SoftDevice is NOT enabled");
    }

    while (true) {
        NRF_LOG_PROCESS();
    }
}

/**
 *@}
 **/
