#ifndef EEZ_LVGL_UI_VARS_H
#define EEZ_LVGL_UI_VARS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// enum declarations

// Flow global variables

enum FlowGlobalVariables {
    FLOW_GLOBAL_VARIABLE_CH1 = 0,
    FLOW_GLOBAL_VARIABLE_CH2 = 1,
    FLOW_GLOBAL_VARIABLE_CH3 = 2,
    FLOW_GLOBAL_VARIABLE_CH4 = 3,
    FLOW_GLOBAL_VARIABLE_CH5 = 4,
    FLOW_GLOBAL_VARIABLE_CH6 = 5,
    FLOW_GLOBAL_VARIABLE_SW1 = 6,
    FLOW_GLOBAL_VARIABLE_SW2 = 7,
    FLOW_GLOBAL_VARIABLE_SW3 = 8,
    FLOW_GLOBAL_VARIABLE_SW4 = 9,
    FLOW_GLOBAL_VARIABLE_LOAD_BAR_PROGRESS = 10,
    FLOW_GLOBAL_VARIABLE_BINDING_LED_COLOR = 11,
    FLOW_GLOBAL_VARIABLE_DEVICE_NAME = 12,
    FLOW_GLOBAL_VARIABLE_DEVICE_LIST = 13,
    FLOW_GLOBAL_VARIABLE_REVERSE1 = 14,
    FLOW_GLOBAL_VARIABLE_REVERSE2 = 15,
    FLOW_GLOBAL_VARIABLE_REVERSE3 = 16,
    FLOW_GLOBAL_VARIABLE_REVERSE4 = 17,
    FLOW_GLOBAL_VARIABLE_CH1_MIN = 18,
    FLOW_GLOBAL_VARIABLE_CH2_MIN = 19,
    FLOW_GLOBAL_VARIABLE_CH3_MIN = 20,
    FLOW_GLOBAL_VARIABLE_CH4_MIN = 21,
    FLOW_GLOBAL_VARIABLE_CH1_MAX = 22,
    FLOW_GLOBAL_VARIABLE_CH2_MAX = 23,
    FLOW_GLOBAL_VARIABLE_CH3_MAX = 24,
    FLOW_GLOBAL_VARIABLE_CH4_MAX = 25
};

// Native global variables

extern const char *get_var_ch1();
extern void set_var_ch1(const char *value);
extern const char *get_var_ch2();
extern void set_var_ch2(const char *value);
extern const char *get_var_ch3();
extern void set_var_ch3(const char *value);
extern const char *get_var_ch4();
extern void set_var_ch4(const char *value);
extern const char *get_var_ch5();
extern void set_var_ch5(const char *value);
extern const char *get_var_ch6();
extern void set_var_ch6(const char *value);
extern bool get_var_sw1();
extern void set_var_sw1(bool value);
extern bool get_var_sw2();
extern void set_var_sw2(bool value);
extern bool get_var_sw3();
extern void set_var_sw3(bool value);
extern bool get_var_sw4();
extern void set_var_sw4(bool value);
extern int32_t get_var_load_bar_progress();
extern void set_var_load_bar_progress(int32_t value);
extern int32_t get_var_binding_led_color();
extern void set_var_binding_led_color(int32_t value);
extern const char *get_var_device_name();
extern void set_var_device_name(const char *value);
extern const char *get_var_device_list();
extern void set_var_device_list(const char *value);
extern bool get_var_reverse1();
extern void set_var_reverse1(bool value);
extern bool get_var_reverse2();
extern void set_var_reverse2(bool value);
extern bool get_var_reverse3();
extern void set_var_reverse3(bool value);
extern bool get_var_reverse4();
extern void set_var_reverse4(bool value);
extern int32_t get_var_ch1_min();
extern void set_var_ch1_min(int32_t value);
extern int32_t get_var_ch2_min();
extern void set_var_ch2_min(int32_t value);
extern int32_t get_var_ch3_min();
extern void set_var_ch3_min(int32_t value);
extern int32_t get_var_ch4_min();
extern void set_var_ch4_min(int32_t value);
extern int32_t get_var_ch1_max();
extern void set_var_ch1_max(int32_t value);
extern int32_t get_var_ch2_max();
extern void set_var_ch2_max(int32_t value);
extern int32_t get_var_ch3_max();
extern void set_var_ch3_max(int32_t value);
extern int32_t get_var_ch4_max();
extern void set_var_ch4_max(int32_t value);

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_VARS_H*/