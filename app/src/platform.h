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

/* Startup delays for each thread */
#define RADIO_STARTUP_DELAY     500
#define ADC_STARTUP_DELAY       500+RADIO_STARTUP_DELAY
#define BUZZER_STARTUP_DELAY    500+ADC_STARTUP_DELAY


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
/* Calibration offset for ADC values */
#define CALIBRATION_OFFSET 40*ADC_MAX_VALUE/255 // Example offset, adjust based on actual calibration results

/* PCF8575 is used with buttons, here is the mapping */
#define TRIM_JOYSTICK_ROULIS_UP    0
#define IS_TRIM_JOYSTICK_ROULIS_UP_ACTIVATED(pins) (((pins) & (1 << TRIM_JOYSTICK_ROULIS_UP)) == 0)
#define TRIM_JOYSTICK_ROULIS_DOWN  1
#define IS_TRIM_JOYSTICK_ROULIS_DOWN_ACTIVATED(pins) (((pins) & (1 << TRIM_JOYSTICK_ROULIS_DOWN)) == 0)
#define TRIM_JOYSTICK_TANGAGE_UP   6
#define IS_TRIM_JOYSTICK_TANGAGE_UP_ACTIVATED(pins) (((pins) & (1 << TRIM_JOYSTICK_TANGAGE_UP)) == 0)
#define TRIM_JOYSTICK_TANGAGE_DOWN 7
#define IS_TRIM_JOYSTICK_TANGAGE_DOWN_ACTIVATED(pins) (((pins) & (1 << TRIM_JOYSTICK_TANGAGE_DOWN)) == 0)
#define TRIM_JOYSTICK_LACET_UP     8
#define IS_TRIM_JOYSTICK_LACET_UP_ACTIVATED(pins) (((pins) & (1 << TRIM_JOYSTICK_LACET_UP)) == 0)
#define TRIM_JOYSTICK_LACET_DOWN   9
#define IS_TRIM_JOYSTICK_LACET_DOWN_ACTIVATED(pins) (((pins) & (1 << TRIM_JOYSTICK_LACET_DOWN)) == 0)
#define TRIM_JOYSTICK_GAZ_UP       2
#define IS_TRIM_JOYSTICK_GAZ_UP_ACTIVATED(pins) (((pins) & (1 << TRIM_JOYSTICK_GAZ_UP)) == 0)
#define TRIM_JOYSTICK_GAZ_DOWN     12
#define IS_TRIM_JOYSTICK_GAZ_DOWN_ACTIVATED(pins) (((pins) & (1 << TRIM_JOYSTICK_GAZ_DOWN)) == 0)

#define TRIM_INCREMENT (2*ADC_MAX_VALUE/255) // 2 steps increment for trimming, adjust as needed

// ON/OFF switches
#define SWITCH_SW_1 5
#define IS_SWITCH_SW_1_ACTIVATED(pins) (((pins) & (1 << SWITCH_SW_1)) == 0)
#define SWITCH_SW_2 10
#define IS_SWITCH_SW_2_ACTIVATED(pins) (((pins) & (1 << SWITCH_SW_2)) == 0)
#define SWITCH_SW_3 11
#define IS_SWITCH_SW_3_ACTIVATED(pins) (((pins) & (1 << SWITCH_SW_3)) == 0)
#define SWITCH_SW_4 13
#define IS_SWITCH_SW_4_ACTIVATED(pins) (((pins) & (1 << SWITCH_SW_4)) == 0)


// Push buttons
#define SWITCH_SW_A_UP      3
#define IS_SWITCH_SW_A_UP_ACTIVATED(pins) (((pins) & (1 << SWITCH_SW_A_UP)) == 0)
#define SWITCH_SW_A_DOWN    4
#define IS_SWITCH_SW_A_DOWN_ACTIVATED(pins) (((pins) & (1 << SWITCH_SW_A_DOWN)) == 0)
#define SWITCH_SW_B_UP      14
#define IS_SWITCH_SW_B_UP_ACTIVATED(pins) (((pins) & (1 << SWITCH_SW_B_UP)) == 0)
#define SWITCH_SW_B_DOWN    15
#define IS_SWITCH_SW_B_DOWN_ACTIVATED(pins) (((pins) & (1 << SWITCH_SW_B_DOWN)) == 0)
