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

/*  ADC Resolution */
#define ADC_RESOLUTION 12
#define ADC_MAX_VALUE ((1 << ADC_RESOLUTION) - 1)

/* PCF8575 is used with buttons, here is the mapping */
#define TRIM_JOYSTICK_ROULIS_UP    0
#define TRIM_JOYSTICK_ROULIS_DOWN  1
#define TRIM_JOYSTICK_TANGAGE_UP   2
#define TRIM_JOYSTICK_TANGAGE_DOWN 3
// #define TRIM_JOYSTICK_GAZ    // useless
#define TRIM_JOYSTICK_LACET_UP     4
#define TRIM_JOYSTICK_LACET_DOWN   5

// ON/OFF switches
#define SWITCH_SW_1 6
#define IS_SWITCH_SW_1_ACTIVATED(pins) (((pins) & (1 << SWITCH_SW_1)) == 0)
#define SWITCH_SW_2 7
#define IS_SWITCH_SW_2_ACTIVATED(pins) (((pins) & (1 << SWITCH_SW_2)) == 0)
#define SWITCH_SW_3 8
#define IS_SWITCH_SW_3_ACTIVATED(pins) (((pins) & (1 << SWITCH_SW_3)) == 0)
#define SWITCH_SW_4 9
#define IS_SWITCH_SW_4_ACTIVATED(pins) (((pins) & (1 << SWITCH_SW_4)) == 0)
#define SWITCH_SW_4_1 10
#define IS_SWITCH_SW_4_1_ACTIVATED(pins) (((pins) & (1 << SWITCH_SW_4_1)) == 0)


// Push buttons
#define SWITCH_PUSH 11
#define SWITCH_PUSH_UP 12
#define SWITCH_PUSH_DOWN 13
