#include <string.h>
#include <lvgl_private.h>
#include "screens.h"
#include "images.h"
#include "fonts.h"
#include "actions.h"
#include "vars.h"
#include "styles.h"
#include "ui.h"

#include <string.h>

objects_t objects;

//
// Event handlers
//

lv_obj_t *tick_value_change_obj;

static void event_handler_cb_settings_tick_sw1(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            bool value = lv_obj_has_state(ta, LV_STATE_CHECKED);
            set_var_sw1(value);
        }
    }
}

static void event_handler_cb_settings_tick_sw3(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            bool value = lv_obj_has_state(ta, LV_STATE_CHECKED);
            set_var_sw3(value);
        }
    }
}

static void event_handler_cb_settings_tick_sw2(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            bool value = lv_obj_has_state(ta, LV_STATE_CHECKED);
            set_var_sw2(value);
        }
    }
}

static void event_handler_cb_settings_tick_sw4(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            bool value = lv_obj_has_state(ta, LV_STATE_CHECKED);
            set_var_sw4(value);
        }
    }
}

static void event_handler_cb_settings_reverse1_switch(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            bool value = lv_obj_has_state(ta, LV_STATE_CHECKED);
            set_var_reverse1(value);
        }
    }
}

static void event_handler_cb_settings_reverse2_switch(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            bool value = lv_obj_has_state(ta, LV_STATE_CHECKED);
            set_var_reverse2(value);
        }
    }
}

static void event_handler_cb_settings_reverse3_switch(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            bool value = lv_obj_has_state(ta, LV_STATE_CHECKED);
            set_var_reverse3(value);
        }
    }
}

static void event_handler_cb_settings_reverse4_switch(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            bool value = lv_obj_has_state(ta, LV_STATE_CHECKED);
            set_var_reverse4(value);
        }
    }
}

static void event_handler_cb_settings_spin_ch1_min(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            int32_t value = lv_spinbox_get_value(ta);
            set_var_ch1_min(value);
        }
    }
}

static void event_handler_cb_settings_spin_ch1_max(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            int32_t value = lv_spinbox_get_value(ta);
            set_var_ch1_max(value);
        }
    }
}

static void event_handler_cb_settings_spin_ch2_min(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            int32_t value = lv_spinbox_get_value(ta);
            set_var_ch2_min(value);
        }
    }
}

static void event_handler_cb_settings_spin_ch2_max(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            int32_t value = lv_spinbox_get_value(ta);
            set_var_ch2_max(value);
        }
    }
}

static void event_handler_cb_settings_spin_ch3_min(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            int32_t value = lv_spinbox_get_value(ta);
            set_var_ch3_min(value);
        }
    }
}

static void event_handler_cb_settings_spin_ch3_max(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            int32_t value = lv_spinbox_get_value(ta);
            set_var_ch3_max(value);
        }
    }
}

static void event_handler_cb_settings_spin_ch4_min(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            int32_t value = lv_spinbox_get_value(ta);
            set_var_ch4_min(value);
        }
    }
}

static void event_handler_cb_settings_spin_ch4_max(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            int32_t value = lv_spinbox_get_value(ta);
            set_var_ch4_max(value);
        }
    }
}

static void event_handler_cb_device_name_model_name_text(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            const char *value = lv_textarea_get_text(ta);
            set_var_device_name(value);
        }
    }
}

//
// Screens
//

void create_screen_main() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.main = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 800, 480);
    {
        lv_obj_t *parent_obj = obj;
        {
            // Binding-Led
            lv_obj_t *obj = lv_led_create(parent_obj);
            objects.binding_led = obj;
            lv_obj_set_pos(obj, 34, 249);
            lv_obj_set_size(obj, 33, 33);
            lv_led_set_brightness(obj, 255);
        }
        {
            // StartButton
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.start_button = obj;
            lv_obj_set_pos(obj, 391, 240);
            lv_obj_set_size(obj, 100, 50);
            lv_obj_add_event_cb(obj, action_start_button_pressed, LV_EVENT_PRESSED, (void *)0);
            {
                lv_obj_t *parent_obj = obj;
                {
                    // ButtonLabel
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.button_label = obj;
                    lv_obj_set_pos(obj, 0, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text_static(obj, "Start");
                }
            }
        }
        {
            // CounterLabel
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.counter_label = obj;
            lv_obj_set_pos(obj, 354, 117);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text(obj, "");
        }
        {
            // Banner
            lv_obj_t *obj = lv_obj_create(parent_obj);
            objects.banner = obj;
            lv_obj_set_pos(obj, 9, 0);
            lv_obj_set_size(obj, 783, 65);
            lv_obj_set_style_pad_left(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_top(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_right(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_bottom(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_opa(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_radius(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_grad_color(obj, lv_color_hex(0xebe0e0), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        {
            // Menu
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.menu = obj;
            lv_obj_set_pos(obj, 0, 65);
            lv_obj_set_size(obj, 119, 120);
            lv_obj_add_event_cb(obj, action_menu_settings_action, LV_EVENT_PRESSED, (void *)0);
            lv_obj_set_style_radius(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_image_src(obj, &img_settings_icon, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xf1c38d), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
    
    tick_screen_main();
}

void delete_screen_main() {
    lv_obj_delete(objects.main);
    objects.main = 0;
    objects.binding_led = 0;
    objects.start_button = 0;
    objects.button_label = 0;
    objects.counter_label = 0;
    objects.banner = 0;
    objects.menu = 0;
}

void tick_screen_main() {
    {
        uint32_t new_val = get_var_binding_led_color();
        new_val = lv_color_to_u32(lv_color_hex(new_val));
        uint32_t cur_val = lv_color_to_u32(((lv_led_t *)objects.binding_led)->color);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.binding_led;
            lv_led_set_color(objects.binding_led, lv_color_hex(new_val));
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = get_var_device_name();
        const char *cur_val = lv_label_get_text(objects.counter_label);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.counter_label;
            lv_label_set_text(objects.counter_label, new_val);
            tick_value_change_obj = NULL;
        }
    }
}

void create_screen_settings() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.settings = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 800, 480);
    {
        lv_obj_t *parent_obj = obj;
        {
            // SettingsTabView
            lv_obj_t *obj = lv_tabview_create(parent_obj);
            objects.settings_tab_view = obj;
            lv_obj_set_pos(obj, 77, 1);
            lv_obj_set_size(obj, 646, 479);
            lv_tabview_set_tab_bar_position(obj, LV_DIR_TOP);
            lv_tabview_set_tab_bar_size(obj, 50);
            lv_tabview_set_active(obj, 1, LV_ANIM_OFF);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0x9294a3), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
            {
                lv_obj_t *parent_obj = obj;
                {
                    // TabButton
                    lv_obj_t *obj = lv_tabview_add_tab(parent_obj, "Buttons");
                    objects.tab_button = obj;
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                    {
                        lv_obj_t *parent_obj = obj;
                        {
                            // Tick-Sw1
                            lv_obj_t *obj = lv_switch_create(parent_obj);
                            objects.tick_sw1 = obj;
                            lv_obj_set_pos(obj, 100, 8);
                            lv_obj_set_size(obj, 50, 25);
                            lv_obj_add_event_cb(obj, event_handler_cb_settings_tick_sw1, LV_EVENT_ALL, 0);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0x42ed08), LV_PART_INDICATOR | LV_STATE_CHECKED);
                        }
                        {
                            // Tick-Sw3
                            lv_obj_t *obj = lv_switch_create(parent_obj);
                            objects.tick_sw3 = obj;
                            lv_obj_set_pos(obj, 100, 181);
                            lv_obj_set_size(obj, 50, 22);
                            lv_obj_add_event_cb(obj, event_handler_cb_settings_tick_sw3, LV_EVENT_ALL, 0);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0x42ed08), LV_PART_INDICATOR | LV_STATE_CHECKED);
                        }
                        {
                            // Sw2
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.sw2 = obj;
                            lv_obj_set_pos(obj, 10, 96);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "SW-2");
                        }
                        {
                            // Sw3
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.sw3 = obj;
                            lv_obj_set_pos(obj, 10, 183);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "SW-3");
                        }
                        {
                            // Sw1
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.sw1 = obj;
                            lv_obj_set_pos(obj, 10, 10);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "SW-1");
                        }
                        {
                            // Tick-Sw2
                            lv_obj_t *obj = lv_switch_create(parent_obj);
                            objects.tick_sw2 = obj;
                            lv_obj_set_pos(obj, 100, 94);
                            lv_obj_set_size(obj, 50, 25);
                            lv_obj_add_event_cb(obj, event_handler_cb_settings_tick_sw2, LV_EVENT_ALL, 0);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0x42ed08), LV_PART_INDICATOR | LV_STATE_CHECKED);
                        }
                        {
                            // Sw4
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.sw4 = obj;
                            lv_obj_set_pos(obj, 10, 269);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "SW-4");
                        }
                        {
                            // Tick-Sw4
                            lv_obj_t *obj = lv_switch_create(parent_obj);
                            objects.tick_sw4 = obj;
                            lv_obj_set_pos(obj, 100, 265);
                            lv_obj_set_size(obj, 50, 25);
                            lv_obj_add_event_cb(obj, event_handler_cb_settings_tick_sw4, LV_EVENT_ALL, 0);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0x42ed08), LV_PART_INDICATOR | LV_STATE_CHECKED);
                        }
                    }
                }
                {
                    // TabChannel
                    lv_obj_t *obj = lv_tabview_add_tab(parent_obj, "Channels");
                    objects.tab_channel = obj;
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                    {
                        lv_obj_t *parent_obj = obj;
                        {
                            // LabelRoulis
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.label_roulis = obj;
                            lv_obj_set_pos(obj, -218, -141);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Ch1 (Ailerons) ");
                        }
                        {
                            // LabelLacet
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.label_lacet = obj;
                            lv_obj_set_pos(obj, -219, 137);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Ch4 (Derive)");
                        }
                        {
                            // LabelTangage
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.label_tangage = obj;
                            lv_obj_set_pos(obj, -219, -49);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Ch2 (Profond.)");
                        }
                        {
                            // LabelGaz
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.label_gaz = obj;
                            lv_obj_set_pos(obj, -219, 41);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Ch3 (Gaz)");
                        }
                        {
                            // ReverseLabel
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.reverse_label = obj;
                            lv_obj_set_pos(obj, 159, -10);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Reverse");
                        }
                        {
                            // Reverse1Switch
                            lv_obj_t *obj = lv_switch_create(parent_obj);
                            objects.reverse1_switch = obj;
                            lv_obj_set_pos(obj, 182, 40);
                            lv_obj_set_size(obj, 50, 25);
                            lv_obj_add_event_cb(obj, event_handler_cb_settings_reverse1_switch, LV_EVENT_ALL, 0);
                            lv_obj_add_state(obj, LV_STATE_FOCUS_KEY);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0x42ed08), LV_PART_INDICATOR | LV_STATE_CHECKED);
                        }
                        {
                            // Reverse2Switch
                            lv_obj_t *obj = lv_switch_create(parent_obj);
                            objects.reverse2_switch = obj;
                            lv_obj_set_pos(obj, 182, 133);
                            lv_obj_set_size(obj, 50, 25);
                            lv_obj_add_event_cb(obj, event_handler_cb_settings_reverse2_switch, LV_EVENT_ALL, 0);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0x42ed08), LV_PART_INDICATOR | LV_STATE_CHECKED);
                        }
                        {
                            // Reverse3Switch
                            lv_obj_t *obj = lv_switch_create(parent_obj);
                            objects.reverse3_switch = obj;
                            lv_obj_set_pos(obj, 182, 225);
                            lv_obj_set_size(obj, 50, 25);
                            lv_obj_add_event_cb(obj, event_handler_cb_settings_reverse3_switch, LV_EVENT_ALL, 0);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0x42ed08), LV_PART_INDICATOR | LV_STATE_CHECKED);
                        }
                        {
                            // Reverse4Switch
                            lv_obj_t *obj = lv_switch_create(parent_obj);
                            objects.reverse4_switch = obj;
                            lv_obj_set_pos(obj, 182, 320);
                            lv_obj_set_size(obj, 50, 25);
                            lv_obj_add_event_cb(obj, event_handler_cb_settings_reverse4_switch, LV_EVENT_ALL, 0);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0x42ed08), LV_PART_INDICATOR | LV_STATE_CHECKED);
                        }
                        {
                            // Lab-Text-Min
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.lab_text_min = obj;
                            lv_obj_set_pos(obj, 302, -9);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Min (%)");
                        }
                        {
                            lv_obj_t *obj = lv_obj_create(parent_obj);
                            lv_obj_set_pos(obj, 250, 25);
                            lv_obj_set_size(obj, 345, 50);
                            lv_obj_set_style_pad_left(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_top(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_right(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_bottom(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_radius(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_bg_opa(obj, 100, LV_PART_MAIN | LV_STATE_DEFAULT);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    // But-Ch1Min-minus
                                    lv_obj_t *obj = lv_button_create(parent_obj);
                                    objects.but_ch1_min_minus = obj;
                                    lv_obj_set_pos(obj, 4, 5);
                                    lv_obj_set_size(obj, 40, 40);
                                    lv_obj_add_event_cb(obj, action_ch1_min_spin_decrement, LV_EVENT_PRESSED, (void *)0);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, 0, 0);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text_static(obj, "-");
                                        }
                                    }
                                }
                                {
                                    // But-Ch1Min-plus
                                    lv_obj_t *obj = lv_button_create(parent_obj);
                                    objects.but_ch1_min_plus = obj;
                                    lv_obj_set_pos(obj, 110, 5);
                                    lv_obj_set_size(obj, 40, 40);
                                    lv_obj_add_event_cb(obj, action_ch1_min_spin_increment, LV_EVENT_PRESSED, (void *)0);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, 0, 0);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text_static(obj, "+");
                                        }
                                    }
                                }
                                {
                                    // Spin-Ch1Min
                                    lv_obj_t *obj = lv_spinbox_create(parent_obj);
                                    objects.spin_ch1_min = obj;
                                    lv_obj_set_pos(obj, 47, 1);
                                    lv_obj_set_size(obj, 60, 49);
                                    lv_spinbox_set_digit_format(obj, 3, 0);
                                    lv_spinbox_set_range(obj, 0, 100);
                                    lv_spinbox_set_rollover(obj, true);
                                    lv_spinbox_set_step(obj, 1);
                                    lv_obj_add_event_cb(obj, event_handler_cb_settings_spin_ch1_min, LV_EVENT_ALL, 0);
                                }
                                {
                                    // But-Ch1Max-minus
                                    lv_obj_t *obj = lv_button_create(parent_obj);
                                    objects.but_ch1_max_minus = obj;
                                    lv_obj_set_pos(obj, 197, 5);
                                    lv_obj_set_size(obj, 40, 40);
                                    lv_obj_add_event_cb(obj, action_ch1_max_decrement, LV_EVENT_PRESSED, (void *)0);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, 0, 0);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text_static(obj, "-");
                                        }
                                    }
                                }
                                {
                                    // But-Ch1Max-plus
                                    lv_obj_t *obj = lv_button_create(parent_obj);
                                    objects.but_ch1_max_plus = obj;
                                    lv_obj_set_pos(obj, 301, 5);
                                    lv_obj_set_size(obj, 40, 40);
                                    lv_obj_add_event_cb(obj, action_ch1_max_increment, LV_EVENT_PRESSED, (void *)0);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, 0, 0);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text_static(obj, "+");
                                        }
                                    }
                                }
                                {
                                    // Spin-Ch1Max
                                    lv_obj_t *obj = lv_spinbox_create(parent_obj);
                                    objects.spin_ch1_max = obj;
                                    lv_obj_set_pos(obj, 240, 1);
                                    lv_obj_set_size(obj, 60, 49);
                                    lv_spinbox_set_digit_format(obj, 3, 0);
                                    lv_spinbox_set_range(obj, 0, 100);
                                    lv_spinbox_set_rollover(obj, true);
                                    lv_spinbox_set_step(obj, 1);
                                    lv_obj_add_event_cb(obj, event_handler_cb_settings_spin_ch1_max, LV_EVENT_ALL, 0);
                                }
                            }
                        }
                        {
                            // Lab-Text-Max
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.lab_text_max = obj;
                            lv_obj_set_pos(obj, 480, -9);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Max (%)");
                        }
                        {
                            lv_obj_t *obj = lv_obj_create(parent_obj);
                            lv_obj_set_pos(obj, 250, 120);
                            lv_obj_set_size(obj, 345, 50);
                            lv_obj_set_style_pad_left(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_top(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_right(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_bottom(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_radius(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_bg_opa(obj, 100, LV_PART_MAIN | LV_STATE_DEFAULT);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    // But-Ch2Min-minus
                                    lv_obj_t *obj = lv_button_create(parent_obj);
                                    objects.but_ch2_min_minus = obj;
                                    lv_obj_set_pos(obj, 4, 5);
                                    lv_obj_set_size(obj, 40, 40);
                                    lv_obj_add_event_cb(obj, action_ch2_min_decrement, LV_EVENT_PRESSED, (void *)0);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, 0, 0);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text_static(obj, "-");
                                        }
                                    }
                                }
                                {
                                    // But-Ch2Min-plus
                                    lv_obj_t *obj = lv_button_create(parent_obj);
                                    objects.but_ch2_min_plus = obj;
                                    lv_obj_set_pos(obj, 110, 5);
                                    lv_obj_set_size(obj, 40, 40);
                                    lv_obj_add_event_cb(obj, action_ch2_min_increment, LV_EVENT_PRESSED, (void *)0);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, 0, 0);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text_static(obj, "+");
                                        }
                                    }
                                }
                                {
                                    // Spin-Ch2Min
                                    lv_obj_t *obj = lv_spinbox_create(parent_obj);
                                    objects.spin_ch2_min = obj;
                                    lv_obj_set_pos(obj, 47, 1);
                                    lv_obj_set_size(obj, 60, 49);
                                    lv_spinbox_set_digit_format(obj, 3, 0);
                                    lv_spinbox_set_range(obj, 0, 100);
                                    lv_spinbox_set_rollover(obj, true);
                                    lv_spinbox_set_step(obj, 1);
                                    lv_obj_add_event_cb(obj, event_handler_cb_settings_spin_ch2_min, LV_EVENT_ALL, 0);
                                }
                                {
                                    // But-Ch2Max-minus
                                    lv_obj_t *obj = lv_button_create(parent_obj);
                                    objects.but_ch2_max_minus = obj;
                                    lv_obj_set_pos(obj, 197, 5);
                                    lv_obj_set_size(obj, 40, 40);
                                    lv_obj_add_event_cb(obj, action_ch2_max_decrement, LV_EVENT_PRESSED, (void *)0);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, 0, 0);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text_static(obj, "-");
                                        }
                                    }
                                }
                                {
                                    // But-Ch2Max-plus
                                    lv_obj_t *obj = lv_button_create(parent_obj);
                                    objects.but_ch2_max_plus = obj;
                                    lv_obj_set_pos(obj, 301, 5);
                                    lv_obj_set_size(obj, 40, 40);
                                    lv_obj_add_event_cb(obj, action_ch2_max_increment, LV_EVENT_PRESSED, (void *)0);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, 0, 0);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text_static(obj, "+");
                                        }
                                    }
                                }
                                {
                                    // Spin-Ch2Max
                                    lv_obj_t *obj = lv_spinbox_create(parent_obj);
                                    objects.spin_ch2_max = obj;
                                    lv_obj_set_pos(obj, 240, 1);
                                    lv_obj_set_size(obj, 60, 49);
                                    lv_spinbox_set_digit_format(obj, 3, 0);
                                    lv_spinbox_set_range(obj, 0, 100);
                                    lv_spinbox_set_rollover(obj, true);
                                    lv_spinbox_set_step(obj, 1);
                                    lv_obj_add_event_cb(obj, event_handler_cb_settings_spin_ch2_max, LV_EVENT_ALL, 0);
                                }
                            }
                        }
                        {
                            lv_obj_t *obj = lv_obj_create(parent_obj);
                            lv_obj_set_pos(obj, 250, 215);
                            lv_obj_set_size(obj, 345, 50);
                            lv_obj_set_style_pad_left(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_top(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_right(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_bottom(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_radius(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_bg_opa(obj, 100, LV_PART_MAIN | LV_STATE_DEFAULT);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    // But-Ch3Min-minus
                                    lv_obj_t *obj = lv_button_create(parent_obj);
                                    objects.but_ch3_min_minus = obj;
                                    lv_obj_set_pos(obj, 4, 5);
                                    lv_obj_set_size(obj, 40, 40);
                                    lv_obj_add_event_cb(obj, action_ch3_min_decrement, LV_EVENT_PRESSED, (void *)0);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, 0, 0);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text_static(obj, "-");
                                        }
                                    }
                                }
                                {
                                    // But-Ch3Min-plus
                                    lv_obj_t *obj = lv_button_create(parent_obj);
                                    objects.but_ch3_min_plus = obj;
                                    lv_obj_set_pos(obj, 110, 5);
                                    lv_obj_set_size(obj, 40, 40);
                                    lv_obj_add_event_cb(obj, action_ch3_min_increment, LV_EVENT_PRESSED, (void *)0);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, 0, 0);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text_static(obj, "+");
                                        }
                                    }
                                }
                                {
                                    // Spin-Ch3Min
                                    lv_obj_t *obj = lv_spinbox_create(parent_obj);
                                    objects.spin_ch3_min = obj;
                                    lv_obj_set_pos(obj, 47, 1);
                                    lv_obj_set_size(obj, 60, 49);
                                    lv_spinbox_set_digit_format(obj, 3, 0);
                                    lv_spinbox_set_range(obj, 0, 100);
                                    lv_spinbox_set_rollover(obj, true);
                                    lv_spinbox_set_step(obj, 1);
                                    lv_obj_add_event_cb(obj, event_handler_cb_settings_spin_ch3_min, LV_EVENT_ALL, 0);
                                }
                                {
                                    // But-Ch3Max-minus
                                    lv_obj_t *obj = lv_button_create(parent_obj);
                                    objects.but_ch3_max_minus = obj;
                                    lv_obj_set_pos(obj, 197, 5);
                                    lv_obj_set_size(obj, 40, 40);
                                    lv_obj_add_event_cb(obj, action_ch3_max_decrement, LV_EVENT_PRESSED, (void *)0);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, 0, 0);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text_static(obj, "-");
                                        }
                                    }
                                }
                                {
                                    // But-Ch3Max-plus
                                    lv_obj_t *obj = lv_button_create(parent_obj);
                                    objects.but_ch3_max_plus = obj;
                                    lv_obj_set_pos(obj, 301, 5);
                                    lv_obj_set_size(obj, 40, 40);
                                    lv_obj_add_event_cb(obj, action_ch3_max_increment, LV_EVENT_PRESSED, (void *)0);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, 0, 0);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text_static(obj, "+");
                                        }
                                    }
                                }
                                {
                                    // Spin-Ch3Max
                                    lv_obj_t *obj = lv_spinbox_create(parent_obj);
                                    objects.spin_ch3_max = obj;
                                    lv_obj_set_pos(obj, 240, 1);
                                    lv_obj_set_size(obj, 60, 49);
                                    lv_spinbox_set_digit_format(obj, 3, 0);
                                    lv_spinbox_set_range(obj, 0, 100);
                                    lv_spinbox_set_rollover(obj, true);
                                    lv_spinbox_set_step(obj, 1);
                                    lv_obj_add_event_cb(obj, event_handler_cb_settings_spin_ch3_max, LV_EVENT_ALL, 0);
                                }
                            }
                        }
                        {
                            lv_obj_t *obj = lv_obj_create(parent_obj);
                            lv_obj_set_pos(obj, 250, 310);
                            lv_obj_set_size(obj, 345, 50);
                            lv_obj_set_style_pad_left(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_top(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_right(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_bottom(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_radius(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_bg_opa(obj, 100, LV_PART_MAIN | LV_STATE_DEFAULT);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    // But-Ch4Min-minus
                                    lv_obj_t *obj = lv_button_create(parent_obj);
                                    objects.but_ch4_min_minus = obj;
                                    lv_obj_set_pos(obj, 4, 5);
                                    lv_obj_set_size(obj, 40, 40);
                                    lv_obj_add_event_cb(obj, action_ch4_min_decrement, LV_EVENT_PRESSED, (void *)0);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, 0, 0);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text_static(obj, "-");
                                        }
                                    }
                                }
                                {
                                    // But-Ch4Min-plus
                                    lv_obj_t *obj = lv_button_create(parent_obj);
                                    objects.but_ch4_min_plus = obj;
                                    lv_obj_set_pos(obj, 110, 5);
                                    lv_obj_set_size(obj, 40, 40);
                                    lv_obj_add_event_cb(obj, action_ch4_min_increment, LV_EVENT_PRESSED, (void *)0);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, 0, 0);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text_static(obj, "+");
                                        }
                                    }
                                }
                                {
                                    // Spin-Ch4Min
                                    lv_obj_t *obj = lv_spinbox_create(parent_obj);
                                    objects.spin_ch4_min = obj;
                                    lv_obj_set_pos(obj, 47, 1);
                                    lv_obj_set_size(obj, 60, 49);
                                    lv_spinbox_set_digit_format(obj, 3, 0);
                                    lv_spinbox_set_range(obj, 0, 100);
                                    lv_spinbox_set_rollover(obj, true);
                                    lv_spinbox_set_step(obj, 1);
                                    lv_obj_add_event_cb(obj, event_handler_cb_settings_spin_ch4_min, LV_EVENT_ALL, 0);
                                }
                                {
                                    // But-Ch4Max-minus
                                    lv_obj_t *obj = lv_button_create(parent_obj);
                                    objects.but_ch4_max_minus = obj;
                                    lv_obj_set_pos(obj, 197, 5);
                                    lv_obj_set_size(obj, 40, 40);
                                    lv_obj_add_event_cb(obj, action_ch4_max_decrement, LV_EVENT_PRESSED, (void *)0);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, 0, 0);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text_static(obj, "-");
                                        }
                                    }
                                }
                                {
                                    // But-Ch4Max-plus
                                    lv_obj_t *obj = lv_button_create(parent_obj);
                                    objects.but_ch4_max_plus = obj;
                                    lv_obj_set_pos(obj, 301, 5);
                                    lv_obj_set_size(obj, 40, 40);
                                    lv_obj_add_event_cb(obj, action_ch4_max_increment, LV_EVENT_PRESSED, (void *)0);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, 0, 0);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text_static(obj, "+");
                                        }
                                    }
                                }
                                {
                                    // Spin-Ch4Max
                                    lv_obj_t *obj = lv_spinbox_create(parent_obj);
                                    objects.spin_ch4_max = obj;
                                    lv_obj_set_pos(obj, 240, 1);
                                    lv_obj_set_size(obj, 60, 49);
                                    lv_spinbox_set_digit_format(obj, 3, 0);
                                    lv_spinbox_set_range(obj, 0, 100);
                                    lv_spinbox_set_rollover(obj, true);
                                    lv_spinbox_set_step(obj, 1);
                                    lv_obj_add_event_cb(obj, event_handler_cb_settings_spin_ch4_max, LV_EVENT_ALL, 0);
                                }
                            }
                        }
                    }
                }
                {
                    // Device
                    lv_obj_t *obj = lv_tabview_add_tab(parent_obj, "Device");
                    objects.device = obj;
                    {
                        lv_obj_t *parent_obj = obj;
                        {
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            lv_obj_set_pos(obj, 0, 30);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Current Device: ");
                        }
                        {
                            // DeviceList
                            lv_obj_t *obj = lv_dropdown_create(parent_obj);
                            objects.device_list = obj;
                            lv_obj_set_pos(obj, 195, 180);
                            lv_obj_set_size(obj, 400, LV_SIZE_CONTENT);
                            lv_dropdown_set_options(obj, "");
                            lv_dropdown_set_selected(obj, 0);
                            lv_obj_add_event_cb(obj, action_load_device_list_changed, LV_EVENT_VALUE_CHANGED, (void *)0);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                        {
                            // LoadDeviceLabel
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.load_device_label = obj;
                            lv_obj_set_pos(obj, 0, 193);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Load Device: ");
                        }
                        {
                            // ButtonNewDevice
                            lv_obj_t *obj = lv_button_create(parent_obj);
                            objects.button_new_device = obj;
                            lv_obj_set_pos(obj, 0, 300);
                            lv_obj_set_size(obj, 160, 50);
                            lv_obj_add_event_cb(obj, action_new_device_add, LV_EVENT_PRESSED, (void *)0);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0x2196f3), LV_PART_MAIN | LV_STATE_DEFAULT);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    lv_obj_set_pos(obj, 0, 0);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_label_set_text_static(obj, "New");
                                }
                            }
                        }
                        {
                            // DeviceDeleteButton
                            lv_obj_t *obj = lv_button_create(parent_obj);
                            objects.device_delete_button = obj;
                            lv_obj_set_pos(obj, 435, 300);
                            lv_obj_set_size(obj, 160, 50);
                            lv_obj_add_event_cb(obj, action_delete_device, LV_EVENT_PRESSED, (void *)0);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0x2196f3), LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_bg_opa(obj, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    lv_obj_set_pos(obj, 0, 0);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_label_set_text_static(obj, "Delete");
                                }
                            }
                        }
                        {
                            // DeviceNameLabel
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.device_name_label = obj;
                            lv_obj_set_pos(obj, 220, 30);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_color(obj, lv_color_hex(0xfafafa), LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_border_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "");
                        }
                        {
                            // ResetSettingsButton
                            lv_obj_t *obj = lv_button_create(parent_obj);
                            objects.reset_settings_button = obj;
                            lv_obj_set_pos(obj, 0, 70);
                            lv_obj_set_size(obj, 400, 50);
                            lv_obj_add_event_cb(obj, action_reset_device_settings, LV_EVENT_PRESSED, (void *)0);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    lv_obj_set_pos(obj, 0, 0);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_label_set_text_static(obj, "Reset Settings");
                                }
                            }
                        }
                        {
                            // DuplicateButton
                            lv_obj_t *obj = lv_button_create(parent_obj);
                            objects.duplicate_button = obj;
                            lv_obj_set_pos(obj, 218, 300);
                            lv_obj_set_size(obj, 160, 50);
                            lv_obj_add_event_cb(obj, action_duplicate_model, LV_EVENT_PRESSED, (void *)0);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    lv_obj_set_pos(obj, 0, 0);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_label_set_text_static(obj, "Duplicate");
                                }
                            }
                        }
                    }
                }
                {
                    // TabTrim
                    lv_obj_t *obj = lv_tabview_add_tab(parent_obj, "Trim");
                    objects.tab_trim = obj;
                    {
                        lv_obj_t *parent_obj = obj;
                        {
                            // LabCh1
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.lab_ch1 = obj;
                            lv_obj_set_pos(obj, -228, -158);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Ch1 (Ailerons) ");
                        }
                        {
                            // LabCh2
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.lab_ch2 = obj;
                            lv_obj_set_pos(obj, -229, -103);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Ch2 (Tangage)");
                        }
                        {
                            // LabCh3
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.lab_ch3 = obj;
                            lv_obj_set_pos(obj, -229, -48);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Ch3 (Gaz)");
                        }
                        {
                            // LabCh4
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.lab_ch4 = obj;
                            lv_obj_set_pos(obj, -228, 6);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Ch4 (Lacet)");
                        }
                        {
                            // LabelCh1
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.label_ch1 = obj;
                            lv_obj_set_pos(obj, -100, -158);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "");
                        }
                        {
                            // LabelCh2
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.label_ch2 = obj;
                            lv_obj_set_pos(obj, -98, -103);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "");
                        }
                        {
                            // LabelCh3
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.label_ch3 = obj;
                            lv_obj_set_pos(obj, -98, -48);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "");
                        }
                        {
                            // LabelCh4
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.label_ch4 = obj;
                            lv_obj_set_pos(obj, -97, 6);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "");
                        }
                        {
                            // LabelCh5
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.label_ch5 = obj;
                            lv_obj_set_pos(obj, -98, 61);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "");
                        }
                        {
                            // LabelCh6
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.label_ch6 = obj;
                            lv_obj_set_pos(obj, -98, 116);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "");
                        }
                        {
                            // LabCh5
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.lab_ch5 = obj;
                            lv_obj_set_pos(obj, 51, 245);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Ch5");
                        }
                        {
                            // LabCh6
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.lab_ch6 = obj;
                            lv_obj_set_pos(obj, 51, 300);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Ch6");
                        }
                        {
                            // Calibration
                            lv_obj_t *obj = lv_obj_create(parent_obj);
                            objects.calibration = obj;
                            lv_obj_set_pos(obj, 300, 0);
                            lv_obj_set_size(obj, 300, 125);
                            lv_obj_set_style_pad_left(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_top(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_right(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_bottom(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_radius(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_border_color(obj, lv_color_hex(0x2f3237), LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0x333436), LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_bg_opa(obj, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    // CalibButton
                                    lv_obj_t *obj = lv_button_create(parent_obj);
                                    objects.calib_button = obj;
                                    lv_obj_set_pos(obj, 10, 40);
                                    lv_obj_set_size(obj, 250, 50);
                                    lv_obj_add_event_cb(obj, action_calib_button_pressed, LV_EVENT_PRESSED, (void *)0);
                                    lv_obj_set_style_bg_color(obj, lv_color_hex(0x2196f3), LV_PART_MAIN | LV_STATE_DEFAULT);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            // CalibButtonLabel
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            objects.calib_button_label = obj;
                                            lv_obj_set_pos(obj, 0, 0);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text_static(obj, "Start");
                                        }
                                    }
                                }
                                {
                                    // CalibLabel
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    objects.calib_label = obj;
                                    lv_obj_set_pos(obj, 10, 5);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_label_set_text_static(obj, "Calibration");
                                }
                                {
                                    // CalibBar
                                    lv_obj_t *obj = lv_bar_create(parent_obj);
                                    objects.calib_bar = obj;
                                    lv_obj_set_pos(obj, 10, 100);
                                    lv_obj_set_size(obj, 250, 18);
                                    lv_obj_set_style_border_color(obj, lv_color_hex(0x2196f3), LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_border_width(obj, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
                                }
                            }
                        }
                    }
                }
            }
        }
        {
            // Menu_Back
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.menu_back = obj;
            lv_obj_set_pos(obj, 0, 1);
            lv_obj_set_size(obj, 76, 201);
            lv_obj_add_event_cb(obj, action_menu_back_action, LV_EVENT_PRESSED, (void *)0);
            lv_obj_set_style_radius(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_image_src(obj, &img_back_icon, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        {
            // Save
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.save = obj;
            lv_obj_set_pos(obj, 0, 266);
            lv_obj_set_size(obj, 76, 114);
            lv_obj_add_event_cb(obj, action_save_button_pressed, LV_EVENT_PRESSED, (void *)0);
            lv_obj_set_style_bg_image_src(obj, &img_save_icon, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_image_opa(obj, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_image_recolor(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_opa(obj, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
    
    tick_screen_settings();
}

void delete_screen_settings() {
    lv_obj_delete(objects.settings);
    objects.settings = 0;
    objects.settings_tab_view = 0;
    objects.tab_button = 0;
    objects.tick_sw1 = 0;
    objects.tick_sw3 = 0;
    objects.sw2 = 0;
    objects.sw3 = 0;
    objects.sw1 = 0;
    objects.tick_sw2 = 0;
    objects.sw4 = 0;
    objects.tick_sw4 = 0;
    objects.tab_channel = 0;
    objects.label_roulis = 0;
    objects.label_lacet = 0;
    objects.label_tangage = 0;
    objects.label_gaz = 0;
    objects.reverse_label = 0;
    objects.reverse1_switch = 0;
    objects.reverse2_switch = 0;
    objects.reverse3_switch = 0;
    objects.reverse4_switch = 0;
    objects.lab_text_min = 0;
    objects.but_ch1_min_minus = 0;
    objects.but_ch1_min_plus = 0;
    objects.spin_ch1_min = 0;
    objects.but_ch1_max_minus = 0;
    objects.but_ch1_max_plus = 0;
    objects.spin_ch1_max = 0;
    objects.lab_text_max = 0;
    objects.but_ch2_min_minus = 0;
    objects.but_ch2_min_plus = 0;
    objects.spin_ch2_min = 0;
    objects.but_ch2_max_minus = 0;
    objects.but_ch2_max_plus = 0;
    objects.spin_ch2_max = 0;
    objects.but_ch3_min_minus = 0;
    objects.but_ch3_min_plus = 0;
    objects.spin_ch3_min = 0;
    objects.but_ch3_max_minus = 0;
    objects.but_ch3_max_plus = 0;
    objects.spin_ch3_max = 0;
    objects.but_ch4_min_minus = 0;
    objects.but_ch4_min_plus = 0;
    objects.spin_ch4_min = 0;
    objects.but_ch4_max_minus = 0;
    objects.but_ch4_max_plus = 0;
    objects.spin_ch4_max = 0;
    objects.device = 0;
    objects.device_list = 0;
    objects.load_device_label = 0;
    objects.button_new_device = 0;
    objects.device_delete_button = 0;
    objects.device_name_label = 0;
    objects.reset_settings_button = 0;
    objects.duplicate_button = 0;
    objects.tab_trim = 0;
    objects.lab_ch1 = 0;
    objects.lab_ch2 = 0;
    objects.lab_ch3 = 0;
    objects.lab_ch4 = 0;
    objects.label_ch1 = 0;
    objects.label_ch2 = 0;
    objects.label_ch3 = 0;
    objects.label_ch4 = 0;
    objects.label_ch5 = 0;
    objects.label_ch6 = 0;
    objects.lab_ch5 = 0;
    objects.lab_ch6 = 0;
    objects.calibration = 0;
    objects.calib_button = 0;
    objects.calib_button_label = 0;
    objects.calib_label = 0;
    objects.calib_bar = 0;
    objects.menu_back = 0;
    objects.save = 0;
}

void tick_screen_settings() {
    {
        bool new_val = get_var_sw1();
        bool cur_val = lv_obj_has_state(objects.tick_sw1, LV_STATE_CHECKED);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.tick_sw1;
            if (new_val) {
                lv_obj_add_state(objects.tick_sw1, LV_STATE_CHECKED);
            } else {
                lv_obj_remove_state(objects.tick_sw1, LV_STATE_CHECKED);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = get_var_sw3();
        bool cur_val = lv_obj_has_state(objects.tick_sw3, LV_STATE_CHECKED);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.tick_sw3;
            if (new_val) {
                lv_obj_add_state(objects.tick_sw3, LV_STATE_CHECKED);
            } else {
                lv_obj_remove_state(objects.tick_sw3, LV_STATE_CHECKED);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = get_var_sw2();
        bool cur_val = lv_obj_has_state(objects.tick_sw2, LV_STATE_CHECKED);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.tick_sw2;
            if (new_val) {
                lv_obj_add_state(objects.tick_sw2, LV_STATE_CHECKED);
            } else {
                lv_obj_remove_state(objects.tick_sw2, LV_STATE_CHECKED);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = get_var_sw4();
        bool cur_val = lv_obj_has_state(objects.tick_sw4, LV_STATE_CHECKED);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.tick_sw4;
            if (new_val) {
                lv_obj_add_state(objects.tick_sw4, LV_STATE_CHECKED);
            } else {
                lv_obj_remove_state(objects.tick_sw4, LV_STATE_CHECKED);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = get_var_reverse1();
        bool cur_val = lv_obj_has_state(objects.reverse1_switch, LV_STATE_CHECKED);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.reverse1_switch;
            if (new_val) {
                lv_obj_add_state(objects.reverse1_switch, LV_STATE_CHECKED);
            } else {
                lv_obj_remove_state(objects.reverse1_switch, LV_STATE_CHECKED);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = get_var_reverse2();
        bool cur_val = lv_obj_has_state(objects.reverse2_switch, LV_STATE_CHECKED);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.reverse2_switch;
            if (new_val) {
                lv_obj_add_state(objects.reverse2_switch, LV_STATE_CHECKED);
            } else {
                lv_obj_remove_state(objects.reverse2_switch, LV_STATE_CHECKED);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = get_var_reverse3();
        bool cur_val = lv_obj_has_state(objects.reverse3_switch, LV_STATE_CHECKED);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.reverse3_switch;
            if (new_val) {
                lv_obj_add_state(objects.reverse3_switch, LV_STATE_CHECKED);
            } else {
                lv_obj_remove_state(objects.reverse3_switch, LV_STATE_CHECKED);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = get_var_reverse4();
        bool cur_val = lv_obj_has_state(objects.reverse4_switch, LV_STATE_CHECKED);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.reverse4_switch;
            if (new_val) {
                lv_obj_add_state(objects.reverse4_switch, LV_STATE_CHECKED);
            } else {
                lv_obj_remove_state(objects.reverse4_switch, LV_STATE_CHECKED);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        int32_t new_val = get_var_ch1_min();
        int32_t cur_val = lv_spinbox_get_value(objects.spin_ch1_min);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.spin_ch1_min;
            lv_spinbox_set_value(objects.spin_ch1_min, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        int32_t new_val = get_var_ch1_max();
        int32_t cur_val = lv_spinbox_get_value(objects.spin_ch1_max);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.spin_ch1_max;
            lv_spinbox_set_value(objects.spin_ch1_max, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        int32_t new_val = get_var_ch2_min();
        int32_t cur_val = lv_spinbox_get_value(objects.spin_ch2_min);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.spin_ch2_min;
            lv_spinbox_set_value(objects.spin_ch2_min, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        int32_t new_val = get_var_ch2_max();
        int32_t cur_val = lv_spinbox_get_value(objects.spin_ch2_max);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.spin_ch2_max;
            lv_spinbox_set_value(objects.spin_ch2_max, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        int32_t new_val = get_var_ch3_min();
        int32_t cur_val = lv_spinbox_get_value(objects.spin_ch3_min);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.spin_ch3_min;
            lv_spinbox_set_value(objects.spin_ch3_min, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        int32_t new_val = get_var_ch3_max();
        int32_t cur_val = lv_spinbox_get_value(objects.spin_ch3_max);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.spin_ch3_max;
            lv_spinbox_set_value(objects.spin_ch3_max, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        int32_t new_val = get_var_ch4_min();
        int32_t cur_val = lv_spinbox_get_value(objects.spin_ch4_min);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.spin_ch4_min;
            lv_spinbox_set_value(objects.spin_ch4_min, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        int32_t new_val = get_var_ch4_max();
        int32_t cur_val = lv_spinbox_get_value(objects.spin_ch4_max);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.spin_ch4_max;
            lv_spinbox_set_value(objects.spin_ch4_max, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = get_var_device_list();
        const char *cur_val = lv_dropdown_get_options(objects.device_list);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.device_list;
            lv_dropdown_set_options(objects.device_list, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = get_var_device_name();
        const char *cur_val = lv_label_get_text(objects.device_name_label);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.device_name_label;
            lv_label_set_text(objects.device_name_label, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = get_var_ch1();
        const char *cur_val = lv_label_get_text(objects.label_ch1);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.label_ch1;
            lv_label_set_text(objects.label_ch1, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = get_var_ch2();
        const char *cur_val = lv_label_get_text(objects.label_ch2);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.label_ch2;
            lv_label_set_text(objects.label_ch2, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = get_var_ch3();
        const char *cur_val = lv_label_get_text(objects.label_ch3);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.label_ch3;
            lv_label_set_text(objects.label_ch3, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = get_var_ch4();
        const char *cur_val = lv_label_get_text(objects.label_ch4);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.label_ch4;
            lv_label_set_text(objects.label_ch4, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = get_var_ch5();
        const char *cur_val = lv_label_get_text(objects.label_ch5);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.label_ch5;
            lv_label_set_text(objects.label_ch5, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = get_var_ch6();
        const char *cur_val = lv_label_get_text(objects.label_ch6);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.label_ch6;
            lv_label_set_text(objects.label_ch6, new_val);
            tick_value_change_obj = NULL;
        }
    }
}

void create_screen_splash_screen() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.splash_screen = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 800, 480);
    {
        lv_obj_t *parent_obj = obj;
        {
            // InitBar
            lv_obj_t *obj = lv_bar_create(parent_obj);
            objects.init_bar = obj;
            lv_obj_set_pos(obj, 205, 343);
            lv_obj_set_size(obj, 391, 27);
            lv_obj_add_event_cb(obj, action_update_init_bar, LV_EVENT_VALUE_CHANGED, (void *)5);
        }
        {
            // WelcomeMsg
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.welcome_msg = obj;
            lv_obj_set_pos(obj, 87, 177);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "Adventure72 remote controller");
        }
    }
    
    tick_screen_splash_screen();
}

void delete_screen_splash_screen() {
    lv_obj_delete(objects.splash_screen);
    objects.splash_screen = 0;
    objects.init_bar = 0;
    objects.welcome_msg = 0;
}

void tick_screen_splash_screen() {
    {
        int32_t new_val = get_var_load_bar_progress();
        int32_t cur_val = lv_bar_get_value(objects.init_bar);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.init_bar;
            lv_bar_set_value(objects.init_bar, new_val, LV_ANIM_ON);
            tick_value_change_obj = NULL;
        }
    }
}

void create_screen_device_name() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.device_name = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 800, 480);
    {
        lv_obj_t *parent_obj = obj;
        {
            // DeviceTextMenu
            lv_obj_t *obj = lv_keyboard_create(parent_obj);
            objects.device_text_menu = obj;
            lv_obj_set_pos(obj, 100, 90);
            lv_obj_set_size(obj, 600, 300);
            lv_obj_set_style_align(obj, LV_ALIGN_DEFAULT, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        {
            // ModelNameText
            lv_obj_t *obj = lv_textarea_create(parent_obj);
            objects.model_name_text = obj;
            lv_obj_set_pos(obj, 100, 20);
            lv_obj_set_size(obj, 592, 70);
            lv_textarea_set_max_length(obj, 128);
            lv_textarea_set_one_line(obj, false);
            lv_textarea_set_password_mode(obj, false);
            lv_obj_add_event_cb(obj, event_handler_cb_device_name_model_name_text, LV_EVENT_ALL, 0);
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        {
            // OKButton
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.ok_button = obj;
            lv_obj_set_pos(obj, 700, 30);
            lv_obj_set_size(obj, 90, 50);
            lv_obj_add_event_cb(obj, action_device_name_ok_button_pressed, LV_EVENT_PRESSED, (void *)0);
            {
                lv_obj_t *parent_obj = obj;
                {
                    // OKButtonText
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.ok_button_text = obj;
                    lv_obj_set_pos(obj, 0, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text_static(obj, "OK");
                }
            }
        }
        {
            // DeviceBackButton
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.device_back_button = obj;
            lv_obj_set_pos(obj, 10, 30);
            lv_obj_set_size(obj, 80, 50);
            lv_obj_add_event_cb(obj, action_device_name_back_button_pressed, LV_EVENT_PRESSED, (void *)0);
            {
                lv_obj_t *parent_obj = obj;
                {
                    // BackButtonText
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.back_button_text = obj;
                    lv_obj_set_pos(obj, 0, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text_static(obj, "<");
                }
            }
        }
    }
    lv_keyboard_set_textarea(objects.device_text_menu, objects.model_name_text);
    
    tick_screen_device_name();
}

void delete_screen_device_name() {
    lv_obj_delete(objects.device_name);
    objects.device_name = 0;
    objects.device_text_menu = 0;
    objects.model_name_text = 0;
    objects.ok_button = 0;
    objects.ok_button_text = 0;
    objects.device_back_button = 0;
    objects.back_button_text = 0;
}

void tick_screen_device_name() {
    {
        const char *new_val = get_var_device_name();
        const char *cur_val = lv_textarea_get_text(objects.model_name_text);
        uint32_t max_length = lv_textarea_get_max_length(objects.model_name_text);
        if (strncmp(new_val, cur_val, max_length) != 0) {
            tick_value_change_obj = objects.model_name_text;
            lv_textarea_set_text(objects.model_name_text, new_val);
            tick_value_change_obj = NULL;
        }
    }
}

typedef void (*create_screen_func_t)();
create_screen_func_t create_screen_funcs[] = {
    create_screen_main,
    create_screen_settings,
    create_screen_splash_screen,
    create_screen_device_name,
};
void create_screen(int screen_index) {
    create_screen_funcs[screen_index]();
}
void create_screen_by_id(enum ScreensEnum screenId) {
    create_screen_funcs[screenId - 1]();
}

typedef void (*delete_screen_func_t)();
delete_screen_func_t delete_screen_funcs[] = {
    delete_screen_main,
    delete_screen_settings,
    delete_screen_splash_screen,
    delete_screen_device_name,
};
void delete_screen(int screen_index) {
    delete_screen_funcs[screen_index]();
}
void delete_screen_by_id(enum ScreensEnum screenId) {
    delete_screen_funcs[screenId - 1]();
}

typedef void (*tick_screen_func_t)();
tick_screen_func_t tick_screen_funcs[] = {
    tick_screen_main,
    tick_screen_settings,
    tick_screen_splash_screen,
    tick_screen_device_name,
};
void tick_screen(int screen_index) {
    if (screen_index >= 0 && screen_index < 4) {
        tick_screen_funcs[screen_index]();
    }
}
void tick_screen_by_id(enum ScreensEnum screenId) {
    tick_screen(screenId - 1);
}

//
// Fonts
//

ext_font_desc_t fonts[] = {
#if LV_FONT_MONTSERRAT_8
    { "MONTSERRAT_8", &lv_font_montserrat_8 },
#endif
#if LV_FONT_MONTSERRAT_10
    { "MONTSERRAT_10", &lv_font_montserrat_10 },
#endif
#if LV_FONT_MONTSERRAT_12
    { "MONTSERRAT_12", &lv_font_montserrat_12 },
#endif
#if LV_FONT_MONTSERRAT_14
    { "MONTSERRAT_14", &lv_font_montserrat_14 },
#endif
#if LV_FONT_MONTSERRAT_16
    { "MONTSERRAT_16", &lv_font_montserrat_16 },
#endif
#if LV_FONT_MONTSERRAT_18
    { "MONTSERRAT_18", &lv_font_montserrat_18 },
#endif
#if LV_FONT_MONTSERRAT_20
    { "MONTSERRAT_20", &lv_font_montserrat_20 },
#endif
#if LV_FONT_MONTSERRAT_22
    { "MONTSERRAT_22", &lv_font_montserrat_22 },
#endif
#if LV_FONT_MONTSERRAT_24
    { "MONTSERRAT_24", &lv_font_montserrat_24 },
#endif
#if LV_FONT_MONTSERRAT_26
    { "MONTSERRAT_26", &lv_font_montserrat_26 },
#endif
#if LV_FONT_MONTSERRAT_28
    { "MONTSERRAT_28", &lv_font_montserrat_28 },
#endif
#if LV_FONT_MONTSERRAT_30
    { "MONTSERRAT_30", &lv_font_montserrat_30 },
#endif
#if LV_FONT_MONTSERRAT_32
    { "MONTSERRAT_32", &lv_font_montserrat_32 },
#endif
#if LV_FONT_MONTSERRAT_34
    { "MONTSERRAT_34", &lv_font_montserrat_34 },
#endif
#if LV_FONT_MONTSERRAT_36
    { "MONTSERRAT_36", &lv_font_montserrat_36 },
#endif
#if LV_FONT_MONTSERRAT_38
    { "MONTSERRAT_38", &lv_font_montserrat_38 },
#endif
#if LV_FONT_MONTSERRAT_40
    { "MONTSERRAT_40", &lv_font_montserrat_40 },
#endif
#if LV_FONT_MONTSERRAT_42
    { "MONTSERRAT_42", &lv_font_montserrat_42 },
#endif
#if LV_FONT_MONTSERRAT_44
    { "MONTSERRAT_44", &lv_font_montserrat_44 },
#endif
#if LV_FONT_MONTSERRAT_46
    { "MONTSERRAT_46", &lv_font_montserrat_46 },
#endif
#if LV_FONT_MONTSERRAT_48
    { "MONTSERRAT_48", &lv_font_montserrat_48 },
#endif
};

//
// Color themes
//

uint32_t active_theme_index = 0;

//
//
//

void create_screens() {

// Set default LVGL theme
    lv_display_t *dispp = lv_display_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
    lv_display_set_theme(dispp, theme);
    
    // Initialize screens
    // Create screens
    create_screen_main();
    create_screen_settings();
    create_screen_splash_screen();
    create_screen_device_name();
}