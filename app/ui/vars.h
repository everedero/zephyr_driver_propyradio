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
    FLOW_GLOBAL_VARIABLE_SELECTION1 = 7
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


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_VARS_H*/