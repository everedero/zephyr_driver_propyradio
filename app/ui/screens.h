#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _objects_t {
    lv_obj_t *main;
    lv_obj_t *settings;
    lv_obj_t *sw_a;
    lv_obj_t *sw_a_1;
    lv_obj_t *sw_a_2;
    lv_obj_t *sw_a_3;
    lv_obj_t *sw_a_4;
    lv_obj_t *sw_a_5;
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
    lv_obj_t *aux1;
    lv_obj_t *tab_switch;
    lv_obj_t *menu_back_label;
    lv_obj_t *menu_back_label_3;
    lv_obj_t *menu_back_label_1;
    lv_obj_t *menu_back_label_2;
    lv_obj_t *channel1;
    lv_obj_t *channel1_1;
    lv_obj_t *channel1_2;
    lv_obj_t *channel1_3;
    lv_obj_t *sw1_1;
    lv_obj_t *sw_2;
    lv_obj_t *sw1;
    lv_obj_t *sw_5;
    lv_obj_t *sw_6;
    lv_obj_t *menu_back;
} objects_t;

extern objects_t objects;

enum ScreensEnum {
    SCREEN_ID_MAIN = 1,
    SCREEN_ID_SETTINGS = 2,
};

void create_screen_main();
void tick_screen_main();

void create_screen_settings();
void tick_screen_settings();

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/