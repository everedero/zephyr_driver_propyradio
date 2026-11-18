/*
 * Copyright (C) 2026 Philippe Peurichard <p.peurichard@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <lvgl.h>
#include "vars.h"
#include "screens.h"
#include "actions.h"
#include "../src/model_fs.h"

#define MAX_STR_LEN 8
#define MAX_DEVICE_NAME_LEN 20

bool sw1;
bool sw2;
bool sw3;
bool sw4;
bool sw5;
bool sw6;

int32_t binding_led_color;

int32_t ch1 = 0;
int32_t ch2 = 0;
int32_t ch3 = 0;
int32_t ch4 = 0;
int32_t ch5 = 0;
int32_t ch6 = 0;
char device_name[MAX_DEVICE_NAME_LEN] = { 0 };
char device_list[MAX_DEVICE_NAME_LEN*6] = { 0 };

bool reverse1;
bool reverse2;
bool reverse3;
bool reverse4;

int32_t ch1_min;
int32_t ch2_min;
int32_t ch3_min;
int32_t ch4_min;

int32_t ch1_max;
int32_t ch2_max;
int32_t ch3_max;
int32_t ch4_max;

extern objects_t objects;

static int32_t clamp_min_percentage(int32_t percentage)
{
    if (percentage < 0) {
        return 0;
    }
    if (percentage > 100) {
        return 100;
    }
    return percentage;
}

static int32_t clamp_max_percentage(int32_t percentage)
{
    if (percentage < 0) {
        return 0;
    }
    if (percentage > 100) {
        return 100;
    }
    return percentage;
}
bool get_var_reverse1() {
    return reverse1;
}

void set_var_reverse1(bool value) {
    reverse1 = value;
}

bool get_var_reverse2() {
    return reverse2;
}

void set_var_reverse2(bool value) {
    reverse2 = value;
}

bool get_var_reverse3() {
    return reverse3;
}

void set_var_reverse3(bool value) {
    reverse3 = value;
}


bool get_var_reverse4() {
    return reverse4;
}

void set_var_reverse4(bool value) {
    reverse4 = value;
}


/*
# get_var_sw1

Returns the current value of the switch sw1.

@return bool The state of sw1.
*/
bool get_var_sw1() {
    return sw1;
}

/*
# get_var_sw2

Returns the current value of the switch sw2.

@return bool The state of sw2.
*/
bool get_var_sw2() {
    return sw2;
}
/*
# get_var_sw3

Returns the current value of the switch sw3.

@return bool The state of sw3.
*/
bool get_var_sw3() {
    return sw3;
}
/*
# get_var_sw4

Returns the current value of the switch sw4.

@return bool The state of sw4.
*/
bool get_var_sw4() {
    return sw4; 
}
/*
# get_var_sw5

Returns the current value of the switch sw5.

@return bool The state of sw5.
*/
bool get_var_sw5() {
    return sw5;
}
/*
# get_var_sw6

Returns the current value of the switch sw6.

@return bool The state of sw6.
*/
bool get_var_sw6() {
    return sw6;
}


/*
# set_var_sw1

Sets the value of the switch sw1.

@param value The new state for sw1.
*/
void set_var_sw1(bool value) {
    sw1 = value;
}
/*
# set_var_sw2

Sets the value of the switch sw2.

@param value The new state for sw2.
*/
void set_var_sw2(bool value) {
    sw2 = value;
}
/*
# set_var_sw3

Sets the value of the switch sw3.

@param value The new state for sw3.
*/
void set_var_sw3(bool value) {
    sw3 = value;
}
/*
# set_var_sw4

Sets the value of the switch sw4.

@param value The new state for sw4.
*/
void set_var_sw4(bool value) {
    sw4 = value;
}
/*
# set_var_sw5

Sets the value of the switch sw5.

@param value The new state for sw5.
*/
void set_var_sw5(bool value) {
    sw5 = value;
}
/*
# set_var_sw6

Sets the value of the switch sw6.

@param value The new state for sw6.
*/
void set_var_sw6(bool value) {
    sw6 = value;
}


/*
# get_var_ch1

Returns the current value of ch1 as a string.

@return const char* The string representation of ch1.
*/
const char *get_var_ch1() {
	static char str_buf[MAX_STR_LEN];
	snprintf(str_buf, sizeof(str_buf), "%d", ch1);
	return (const char *) str_buf;
}


/*
# get_var_ch2

Returns the current value of ch2 as a string.

@return const char* The string representation of ch2.
*/
const char *get_var_ch2() {
	static char str_buf[MAX_STR_LEN];
	snprintf(str_buf, sizeof(str_buf), "%d", ch2);
	return (const char *) str_buf;
}


/*
# get_var_ch3

Returns the current value of ch3 as a string.

@return const char* The string representation of ch3.
*/
const char *get_var_ch3() {
	static char str_buf[MAX_STR_LEN];
	snprintf(str_buf, sizeof(str_buf), "%d", ch3);
	return (const char *) str_buf;
}

/*
# get_var_ch4

Returns the current value of ch4 as a string.

@return const char* The string representation of ch4.
*/
const char *get_var_ch4() {
	static char str_buf[MAX_STR_LEN];
	snprintf(str_buf, sizeof(str_buf), "%d", ch4);
	return (const char *) str_buf;
}

/*
# get_var_ch5

Returns the current value of ch5 as a string.

@return const char* The string representation of ch5.
*/
const char *get_var_ch5() {
	static char str_buf[MAX_STR_LEN];
	snprintf(str_buf, sizeof(str_buf), "%d", ch5);
	return (const char *) str_buf;
}

/*
# get_var_ch6

Returns the current value of ch6 as a string.

@return const char* The string representation of ch6.
*/
const char *get_var_ch6() {
	static char str_buf[MAX_STR_LEN];
	snprintf(str_buf, sizeof(str_buf), "%d", ch6);
	return (const char *) str_buf;
}

/*
# set_var_ch1

Sets the value of ch1 from a string. Currently not implemented.

@param value The string value to set (ignored).
*/
void set_var_ch1(const char *value) {}
/*
# set_var_ch2

Sets the value of ch2 from a string. Currently not implemented.

@param value The string value to set (ignored).
*/
void set_var_ch2(const char *value) {}
/*
# set_var_ch3

Sets the value of ch3 from a string. Currently not implemented.

@param value The string value to set (ignored).
*/
void set_var_ch3(const char *value) {}
/*
# set_var_ch4

Sets the value of ch4 from a string. Currently not implemented.

@param value The string value to set (ignored).
*/
void set_var_ch4(const char *value) {}
/*
# set_var_ch5

Sets the value of ch5 from a string. Currently not implemented.

@param value The string value to set (ignored).
*/
void set_var_ch5(const char *value) {}
/*
# set_var_ch6

Sets the value of ch6 from a string. Currently not implemented.

@param value The string value to set (ignored).
*/
void set_var_ch6(const char *value) {}

/*
# set_var_ch1_int

Sets the integer value of ch1.

@param value The new integer value for ch1.
*/
void set_var_ch1_int(int32_t value) {ch1 = value;}
/*
# set_var_ch2_int

Sets the integer value of ch2.

@param value The new integer value for ch2.
*/
void set_var_ch2_int(int32_t value) {ch2 = value;}
/*
# set_var_ch3_int

Sets the integer value of ch3.

@param value The new integer value for ch3.
*/
void set_var_ch3_int(int32_t value) {ch3 = value;}
/*
# set_var_ch4_int

Sets the integer value of ch4.

@param value The new integer value for ch4.
*/
void set_var_ch4_int(int32_t value) {ch4 = value;}
/*
# set_var_ch5_int

Sets the integer value of ch5.

@param value The new integer value for ch5.
*/
void set_var_ch5_int(int32_t value) {ch5 = value;}
/*
# set_var_ch6_int

Sets the integer value of ch6.

@param value The new integer value for ch6.
*/
void set_var_ch6_int(int32_t value) {ch6 = value;}

int32_t load_bar_progress;

int32_t get_var_load_bar_progress() {
    return load_bar_progress;
}

void set_var_load_bar_progress(int32_t value) {
	if (value >= 0 && value <= 100) {
		load_bar_progress = value;
	}
}

int32_t get_var_binding_led_color() {
    return binding_led_color;
}

void set_var_binding_led_color(int32_t value) {
    binding_led_color = value;
}

const char *get_var_device_list() {
    return device_list;
}

static void refresh_device_list() {
    // Clear the device list
    memset(device_list, 0, sizeof(device_list));
    char buf[MODEL_MAX_COUNT][MODEL_NAME_MAX] = {0};
    size_t models_num = 0;

    int rc = model_fs_list_models(buf, &models_num);
    if (rc != 0) {
        return;
    }

    // Iterate through the model storage and append used device names to the list
    for (int i = 0; i < models_num; i++) {
        if (strlen(buf[i]) > 0) {
            if (strlen(device_list) + strlen(buf[i]) + 1 < sizeof(device_list)) {
                strcat(device_list, buf[i]);
                strcat(device_list, "\n");
            }
        }
    }
}

void set_var_device_list(const char *value) {
    refresh_device_list();
}


const char *get_var_device_name() {
    return device_name;
}

void set_var_device_name(const char *value) {
    strncpy(device_name, value, sizeof(device_name) / sizeof(char));
    device_name[sizeof(device_name) / sizeof(char) - 1] = 0;
}

void action_ch1_min_spin_decrement(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_PRESSED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_decrement(objects.spin_ch1_min);
    }
}

void action_ch1_min_spin_increment(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_PRESSED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_increment(objects.spin_ch1_min);
    }
}

void action_ch1_max_decrement(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_PRESSED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_decrement(objects.spin_ch1_max);
    }
}

void action_ch1_max_increment(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_PRESSED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_increment(objects.spin_ch1_max);
    }
}

void action_ch2_min_decrement(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_PRESSED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_decrement(objects.spin_ch2_min);
    }
}

void action_ch2_min_increment(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_PRESSED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_increment(objects.spin_ch2_min);
    }
}

void action_ch2_max_decrement(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_PRESSED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_decrement(objects.spin_ch2_max);
    }
}

void action_ch2_max_increment(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_PRESSED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_increment(objects.spin_ch2_max);
    }
}

void action_ch3_min_decrement(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_PRESSED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_decrement(objects.spin_ch3_min);
    }
}

void action_ch3_min_increment(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_PRESSED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_increment(objects.spin_ch3_min);
    }
}

void action_ch3_max_decrement(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_PRESSED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_decrement(objects.spin_ch3_max);
    }
}

void action_ch3_max_increment(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_PRESSED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_increment(objects.spin_ch3_max);
    }
}

void action_ch4_min_decrement(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_PRESSED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_decrement(objects.spin_ch4_min);
    }
}

void action_ch4_min_increment(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_PRESSED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_increment(objects.spin_ch4_min);
    }
}

void action_ch4_max_decrement(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_PRESSED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_decrement(objects.spin_ch4_max);
    }
}

void action_ch4_max_increment(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_PRESSED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_increment(objects.spin_ch4_max);
    }
}
int32_t get_var_ch1_min() {
    return ch1_min;
}

void set_var_ch1_min(int32_t value) {
    ch1_min = clamp_min_percentage(value);
}

int32_t get_var_ch2_min() {
    return ch2_min;
}

void set_var_ch2_min(int32_t value) {
    ch2_min = clamp_min_percentage(value);
}

int32_t get_var_ch3_min() {
    return ch3_min;
}

void set_var_ch3_min(int32_t value) {
    ch3_min = clamp_min_percentage(value);
}

int32_t get_var_ch4_min() {
    return ch4_min;
}

void set_var_ch4_min(int32_t value) {
    ch4_min = clamp_min_percentage(value);
}


int32_t get_var_ch1_max() {
    return ch1_max;
}

void set_var_ch1_max(int32_t value) {
    ch1_max = clamp_max_percentage(value);
}

int32_t get_var_ch2_max() {
    return ch2_max;
}

void set_var_ch2_max(int32_t value) {
    ch2_max = clamp_max_percentage(value);
}

int32_t get_var_ch3_max() {
    return ch3_max;
}

void set_var_ch3_max(int32_t value) {
    ch3_max = clamp_max_percentage(value);
}

int32_t get_var_ch4_max() {
    return ch4_max;
}

void set_var_ch4_max(int32_t value) {
    ch4_max = clamp_max_percentage(value);
}