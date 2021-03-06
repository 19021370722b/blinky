/*
 * this is the board definition for the breadboard BK throttle
 *
 */


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
#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

#define BOARD_PCA10040

#define BK_THROTTLE_1               20171106

#define BK_SPEED_POT                30

#define BK_DIRECTION_LEFT           11
#define BK_DIRECTION_RIGHT          27

#define BK_BUTTON_BRAKE             15
#define BK_LED_BRAKE                19

#define BK_BUTTON_1                 14
#define BK_BUTTON_2                 8

// make boards.c/bsp.c happy
#define BSP_BUTTON_0                BK_DIRECTION_LEFT
#define BSP_BUTTON_1                BK_DIRECTION_RIGHT
#define BSP_BUTTON_2                BK_BUTTON_1
#define BSP_BUTTON_3                BK_BUTTON_2

#define BUTTON_1 BSP_BUTTON_2
#define BUTTON_2 BSP_BUTTON_3
#define BUTTON_3 BSP_BUTTON_0
#define BUTTON_4 BSP_BUTTON_1

#define BK_LED_1                    2
#define BK_LED_2                    3
#define BK_LED_3                    4
#define BK_LED_4                    5
#define BK_LED_5                    28
#define BK_LED_6                    29
#define BK_LED_7                    12
#define BK_LED_8                    13

#define BK_BATTERY_LEVEL            31

#define BK_NFC1                     9
#define BK_NFC2                     10

#define PILOT_LIGHT                 17

// reserved for future expansion
#define I2C_SCL                     26
#define I2C_SDA                     25

#define LEDS_NUMBER 8
#define LEDS_LIST   { BK_LED_1, BK_LED_2, BK_LED_3, \
                      BK_LED_4, BK_LED_5, BK_LED_6, BK_LED_7, BK_LED_8 }

#define BUTTONS_NUMBER 4
#define BUTTONS_LIST { BK_BUTTON_1, BK_BUTTON_2, BK_DIRECTION_LEFT, BK_DIRECTION_RIGHT }


#define LEDS_ACTIVE_STATE 1

#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source       = NRF_CLOCK_LF_SRC_XTAL,      \
                                 .rc_ctiv      = 0,                          \
                                 .rc_temp_ctiv = 0,                          \
                                 .accuracy     = NRF_CLOCK_LF_ACCURACY_20_PPM}

#ifdef __cplusplus
}
#endif

#endif // CUSTOM_BOARD_H
