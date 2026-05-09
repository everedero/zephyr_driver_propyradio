/*
 * Copyright (C) 2026 Philippe Peurichard <p.peurichard@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */


/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling defaullt priority used for thread */
#define PRIORITY_BUZZER 7
#define PRIORITY_RADIO 6
#define PRIORITY_ADC 5

/* Maximum number of channels */
#define MAX_CHANNELS 6

/* Maximum number of auxiliary channels */
#define MAX_AUX_CHANNELS 4

/* Debounce time in milliseconds */
#define DEBOUNCE_TIME_MS 20

/* Payload size for NRF24L01+ communication */
#define PAYLOAD_SIZE 32

/* PCF8575 is used with buttons, here is the mapping */
#define TRIM_JOYSTICK_ROULIS_UP 0
#define TRIM_JOYSTICK_ROULIS_DOWN 1
#define TRIM_JOYSTICK_TANGAGE_UP 2
#define TRIM_JOYSTICK_TANGAGE_DOWN 3
#define TRIM_JOYSTICK_GAZ_UP 4
#define TRIM_JOYSTICK_GAZ_DOWN 5
#define TRIM_JOYSTICK_LACET_UP 6
#define TRIM_JOYSTICK_LACET_DOWN 7

#define SWITCH_SW_ON_OFF_1 8
#define SWITCH_SW_ON_OFF_2 9
#define SWITCH_SW_3_POSITION_UP 10
#define SWITCH_SW_3_POSITION_DOWN 11
#define SWITCH_SW_4_POSITION_UP 12
#define SWITCH_SW_4_POSITION_DOWN 13

#define SWITCH_SW_5_UP 14
#define SWITCH_SW_5_DOWN 15
#define SWITCH_SW_6_PUSH 16
