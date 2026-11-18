#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

// Screens

enum ScreensEnum {
    _SCREEN_ID_FIRST = 1,
    SCREEN_ID_MAIN = 1,
    SCREEN_ID_SETTINGS = 2,
    SCREEN_ID_SPLASH_SCREEN = 3,
    SCREEN_ID_DEVICE_NAME = 4,
    _SCREEN_ID_LAST = 4
};

typedef struct _objects_t {
    lv_obj_t *main;
    lv_obj_t *settings;
    lv_obj_t *splash_screen;
    lv_obj_t *device_name;
    lv_obj_t *binding_led;
    lv_obj_t *start_button;
    lv_obj_t *button_label;
    lv_obj_t *counter_label;
    lv_obj_t *banner;
    lv_obj_t *menu;
    lv_obj_t *settings_tab_view;
    lv_obj_t *tab_button;
    lv_obj_t *tick_sw1;
    lv_obj_t *tick_sw3;
    lv_obj_t *sw2;
    lv_obj_t *sw3;
    lv_obj_t *sw1;
    lv_obj_t *tick_sw2;
    lv_obj_t *sw4;
    lv_obj_t *tick_sw4;
    lv_obj_t *tab_channel;
    lv_obj_t *label_roulis;
    lv_obj_t *label_lacet;
    lv_obj_t *label_tangage;
    lv_obj_t *label_gaz;
    lv_obj_t *reverse_label;
    lv_obj_t *reverse1_switch;
    lv_obj_t *reverse2_switch;
    lv_obj_t *reverse3_switch;
    lv_obj_t *reverse4_switch;
    lv_obj_t *lab_text_min;
    lv_obj_t *but_ch1_min_minus;
    lv_obj_t *but_ch1_min_plus;
    lv_obj_t *spin_ch1_min;
    lv_obj_t *but_ch1_max_minus;
    lv_obj_t *but_ch1_max_plus;
    lv_obj_t *spin_ch1_max;
    lv_obj_t *lab_text_max;
    lv_obj_t *but_ch2_min_minus;
    lv_obj_t *but_ch2_min_plus;
    lv_obj_t *spin_ch2_min;
    lv_obj_t *but_ch2_max_minus;
    lv_obj_t *but_ch2_max_plus;
    lv_obj_t *spin_ch2_max;
    lv_obj_t *but_ch3_min_minus;
    lv_obj_t *but_ch3_min_plus;
    lv_obj_t *spin_ch3_min;
    lv_obj_t *but_ch3_max_minus;
    lv_obj_t *but_ch3_max_plus;
    lv_obj_t *spin_ch3_max;
    lv_obj_t *but_ch4_min_minus;
    lv_obj_t *but_ch4_min_plus;
    lv_obj_t *spin_ch4_min;
    lv_obj_t *but_ch4_max_minus;
    lv_obj_t *but_ch4_max_plus;
    lv_obj_t *spin_ch4_max;
    lv_obj_t *device;
    lv_obj_t *device_list;
    lv_obj_t *load_device_label;
    lv_obj_t *button_new_device;
    lv_obj_t *device_delete_button;
    lv_obj_t *device_name_label;
    lv_obj_t *reset_settings_button;
    lv_obj_t *duplicate_button;
    lv_obj_t *tab_trim;
    lv_obj_t *lab_ch1;
    lv_obj_t *lab_ch2;
    lv_obj_t *lab_ch3;
    lv_obj_t *lab_ch4;
    lv_obj_t *label_ch1;
    lv_obj_t *label_ch2;
    lv_obj_t *label_ch3;
    lv_obj_t *label_ch4;
    lv_obj_t *label_ch5;
    lv_obj_t *label_ch6;
    lv_obj_t *lab_ch5;
    lv_obj_t *lab_ch6;
    lv_obj_t *calibration;
    lv_obj_t *calib_button;
    lv_obj_t *calib_button_label;
    lv_obj_t *calib_label;
    lv_obj_t *calib_bar;
    lv_obj_t *menu_back;
    lv_obj_t *save;
    lv_obj_t *init_bar;
    lv_obj_t *welcome_msg;
    lv_obj_t *device_text_menu;
    lv_obj_t *model_name_text;
    lv_obj_t *ok_button;
    lv_obj_t *ok_button_text;
    lv_obj_t *device_back_button;
    lv_obj_t *back_button_text;
} objects_t;

extern objects_t objects;

void create_screen_main();
void delete_screen_main();
void tick_screen_main();

void create_screen_settings();
void delete_screen_settings();
void tick_screen_settings();

void create_screen_splash_screen();
void delete_screen_splash_screen();
void tick_screen_splash_screen();

void create_screen_device_name();
void delete_screen_device_name();
void tick_screen_device_name();

void create_screen_by_id(enum ScreensEnum screenId);
void delete_screen_by_id(enum ScreensEnum screenId);
void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/