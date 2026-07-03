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
    _SCREEN_ID_LAST = 3
};

typedef struct _objects_t {
    lv_obj_t *main;
    lv_obj_t *settings;
    lv_obj_t *splash_screen;
    lv_obj_t *binding_led;
    lv_obj_t *label_counter;
    lv_obj_t *start_button;
    lv_obj_t *button_label;
    lv_obj_t *counter_label;
    lv_obj_t *banner;
    lv_obj_t *label_ch1;
    lv_obj_t *label_ch2;
    lv_obj_t *label_ch3;
    lv_obj_t *label_ch4;
    lv_obj_t *label_ch5;
    lv_obj_t *label_ch6;
    lv_obj_t *menu;
    lv_obj_t *settings_tab_view;
    lv_obj_t *tab_radio;
    lv_obj_t *tab_switch;
    lv_obj_t *menu_back_label;
    lv_obj_t *menu_back_label_3;
    lv_obj_t *menu_back_label_1;
    lv_obj_t *menu_back_label_2;
    lv_obj_t *channel1;
    lv_obj_t *channel1_1;
    lv_obj_t *channel1_2;
    lv_obj_t *channel1_3;
    lv_obj_t *sw_a;
    lv_obj_t *sw_a_1;
    lv_obj_t *sw_a_2;
    lv_obj_t *sw1_1;
    lv_obj_t *sw_2;
    lv_obj_t *sw1;
    lv_obj_t *sw_a_5;
    lv_obj_t *device;
    lv_obj_t *device_list;
    lv_obj_t *load_device_label;
    lv_obj_t *obj0;
    lv_obj_t *obj1;
    lv_obj_t *obj2;
    lv_obj_t *obj3;
    lv_obj_t *device_name_label;
    lv_obj_t *menu_back;
    lv_obj_t *save;
    lv_obj_t *init_bar;
    lv_obj_t *welcome_msg;
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

void create_screen_by_id(enum ScreensEnum screenId);
void delete_screen_by_id(enum ScreensEnum screenId);
void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/