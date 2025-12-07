#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _objects_t {
    lv_obj_t *main;
    lv_obj_t *settings;
    lv_obj_t *label_counter;
    lv_obj_t *start_button;
    lv_obj_t *button_label;
    lv_obj_t *counter_label;
    lv_obj_t *banner;
    lv_obj_t *menu;
    lv_obj_t *menu_label;
    lv_obj_t *settings_tab_view;
    lv_obj_t *tab_radio;
    lv_obj_t *tab_switch;
    lv_obj_t *menu_back;
    lv_obj_t *menu_back_label;
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