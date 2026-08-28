#ifndef EEZ_LVGL_UI_VARS_H
#define EEZ_LVGL_UI_VARS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// enum declarations

typedef enum {
    UI_CHANNEL_TAB_Roulis = 0,
    UI_CHANNEL_TAB_Tangage = 1,
    UI_CHANNEL_TAB_Gaz = 2,
    UI_CHANNEL_TAB_Lacet = 3
} UI_CHANNEL_TAB;

// Flow global variables

enum FlowGlobalVariables {
    FLOW_GLOBAL_VARIABLE_COUNTER = 0,
    FLOW_GLOBAL_VARIABLE_CH1 = 1,
    FLOW_GLOBAL_VARIABLE_CH2 = 2,
    FLOW_GLOBAL_VARIABLE_CH3 = 3,
    FLOW_GLOBAL_VARIABLE_CH4 = 4,
    FLOW_GLOBAL_VARIABLE_CH5 = 5,
    FLOW_GLOBAL_VARIABLE_CH6 = 6,
    FLOW_GLOBAL_VARIABLE_SELECTION1 = 7,
    FLOW_GLOBAL_VARIABLE_SW1 = 8,
    FLOW_GLOBAL_VARIABLE_SW2 = 9,
    FLOW_GLOBAL_VARIABLE_SW3 = 10,
    FLOW_GLOBAL_VARIABLE_SW4 = 11,
    FLOW_GLOBAL_VARIABLE_LOAD_BAR_PROGRESS = 12,
    FLOW_GLOBAL_VARIABLE_SELECTION2 = 13,
    FLOW_GLOBAL_VARIABLE_SELECTION3 = 14,
    FLOW_GLOBAL_VARIABLE_SELECTION4 = 15,
    FLOW_GLOBAL_VARIABLE_BINDING_LED_COLOR = 16,
    FLOW_GLOBAL_VARIABLE_DEVICE_NAME = 17,
    FLOW_GLOBAL_VARIABLE_DEVICE_LIST = 18
};

// Native global variables

extern const char *get_var_counter();
extern void set_var_counter(const char *value);
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
extern UI_CHANNEL_TAB get_var_selection1();
extern void set_var_selection1(UI_CHANNEL_TAB value);
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
extern UI_CHANNEL_TAB get_var_selection2();
extern void set_var_selection2(UI_CHANNEL_TAB value);
extern UI_CHANNEL_TAB get_var_selection3();
extern void set_var_selection3(UI_CHANNEL_TAB value);
extern UI_CHANNEL_TAB get_var_selection4();
extern void set_var_selection4(UI_CHANNEL_TAB value);
extern int32_t get_var_binding_led_color();
extern void set_var_binding_led_color(int32_t value);
extern const char *get_var_device_name();
extern void set_var_device_name(const char *value);
extern const char *get_var_device_list();
extern void set_var_device_list(const char *value);

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_VARS_H*/