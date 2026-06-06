/*
 * Copyright (C) 2026 Philippe Peurichard <p.peurichard@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "vars.h"

bool sw1;
bool sw2;
bool sw3;
bool sw4;
bool sw5;
bool sw6;


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

#define MAX_STR_LEN 8
int32_t counter = 0;
int32_t ch1 = 0;
int32_t ch2 = 0;
int32_t ch3 = 0;
int32_t ch4 = 0;
int32_t ch5 = 0;
int32_t ch6 = 0;
UI_CHANNEL_TAB selection1;

/*
# get_var_selection1

Returns the current value of selection1.

@return UI_CHANNEL_TAB The current selection.
*/
UI_CHANNEL_TAB get_var_selection1() {
    return selection1;
}

/*
# set_var_selection1

Sets the value of selection1.

@param value The new value for selection1.
*/
void set_var_selection1(UI_CHANNEL_TAB value) {
    selection1 = value;
}

/*
# get_var_counter

Returns the current value of counter as a string.

@return const char* The string representation of counter.
*/
const char *get_var_counter() {
	static char str_buf[MAX_STR_LEN];
	snprintf(str_buf, sizeof(str_buf), "%d", counter);
	return (const char *) str_buf;}

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
# get_var_counter_int

Returns the current integer value of counter.

@return int32_t The value of counter.
*/
int32_t get_var_counter_int(void)
{
    return counter;
}
/*
# set_var_counter

Sets the value of counter from a string. Currently not implemented.

@param value The string value to set (ignored).
*/
void set_var_counter(const char *value) {}
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
/*
# set_var_counter_int

Sets the integer value of counter.

@param value The new integer value for counter.
*/
void set_var_counter_int(int32_t value) {counter = value;}

int32_t load_bar_progress;

int32_t get_var_load_bar_progress() {
    return load_bar_progress;
}

void set_var_load_bar_progress(int32_t value) {
	if (value >= 0 && value <= 100) {
		load_bar_progress = value;
	}
}
