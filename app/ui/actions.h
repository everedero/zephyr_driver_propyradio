#ifndef EEZ_LVGL_UI_EVENTS_H
#define EEZ_LVGL_UI_EVENTS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void action_start_button_pressed(lv_event_t * e);
extern void action_menu_back_action(lv_event_t * e);
extern void action_menu_settings_action(lv_event_t * e);
extern void action_update_init_bar(lv_event_t * e);
extern void action_save_button_pressed(lv_event_t * e);
extern void action_new_device_add(lv_event_t * e);
extern void action_delete_device(lv_event_t * e);
extern void action_device_name_ok_button_pressed(lv_event_t * e);
extern void action_device_name_back_button_pressed(lv_event_t * e);
extern void action_load_device_list_changed(lv_event_t * e);
extern void action_ch1_min_spin_decrement(lv_event_t * e);
extern void action_ch1_min_spin_increment(lv_event_t * e);
extern void action_ch1_max_decrement(lv_event_t * e);
extern void action_ch1_max_increment(lv_event_t * e);
extern void action_ch2_min_decrement(lv_event_t * e);
extern void action_ch2_min_increment(lv_event_t * e);
extern void action_ch2_max_decrement(lv_event_t * e);
extern void action_ch2_max_increment(lv_event_t * e);
extern void action_ch3_min_decrement(lv_event_t * e);
extern void action_ch3_min_increment(lv_event_t * e);
extern void action_ch3_max_decrement(lv_event_t * e);
extern void action_ch4_max_increment(lv_event_t * e);
extern void action_ch3_max_increment(lv_event_t * e);
extern void action_ch4_min_decrement(lv_event_t * e);
extern void action_ch4_min_increment(lv_event_t * e);
extern void action_ch4_max_decrement(lv_event_t * e);
extern void action_calib_button_pressed(lv_event_t * e);
extern void action_reset_device_settings(lv_event_t * e);
extern void action_duplicate_model(lv_event_t * e);

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_EVENTS_H*/