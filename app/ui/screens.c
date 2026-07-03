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

static void event_handler_cb_settings_channel1(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            int32_t value = lv_dropdown_get_selected(ta);
            set_var_selection1(value);
        }
    }
}

static void event_handler_cb_settings_channel1_1(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            int32_t value = lv_dropdown_get_selected(ta);
            set_var_selection2(value);
        }
    }
}

static void event_handler_cb_settings_channel1_2(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            int32_t value = lv_dropdown_get_selected(ta);
            set_var_selection3(value);
        }
    }
}

static void event_handler_cb_settings_channel1_3(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            int32_t value = lv_dropdown_get_selected(ta);
            set_var_selection4(value);
        }
    }
}

static void event_handler_cb_settings_sw_a(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            bool value = lv_obj_has_state(ta, LV_STATE_CHECKED);
            set_var_sw1(value);
        }
    }
}

static void event_handler_cb_settings_sw_a_1(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            bool value = lv_obj_has_state(ta, LV_STATE_CHECKED);
            set_var_sw3(value);
        }
    }
}

static void event_handler_cb_settings_sw_a_2(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            bool value = lv_obj_has_state(ta, LV_STATE_CHECKED);
            set_var_sw4(value);
        }
    }
}

static void event_handler_cb_settings_sw_a_5(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target_obj(e);
        if (tick_value_change_obj != ta) {
            bool value = lv_obj_has_state(ta, LV_STATE_CHECKED);
            set_var_sw2(value);
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
            // Label_Counter
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.label_counter = obj;
            lv_obj_set_pos(obj, 317, 117);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text_static(obj, "Counter: ");
        }
        {
            // StartButton
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.start_button = obj;
            lv_obj_set_pos(obj, 350, 240);
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
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text_static(obj, "Start");
                }
            }
        }
        {
            // CounterLabel
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.counter_label = obj;
            lv_obj_set_pos(obj, 436, 117);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
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
            {
                lv_obj_t *parent_obj = obj;
                {
                    // LabelCh1
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.label_ch1 = obj;
                    lv_obj_set_pos(obj, -330, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "");
                }
                {
                    // LabelCh2
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.label_ch2 = obj;
                    lv_obj_set_pos(obj, -223, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "");
                }
                {
                    // LabelCh3
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.label_ch3 = obj;
                    lv_obj_set_pos(obj, -117, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "");
                }
                {
                    // LabelCh4
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.label_ch4 = obj;
                    lv_obj_set_pos(obj, -10, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "");
                }
                {
                    // LabelCh5
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.label_ch5 = obj;
                    lv_obj_set_pos(obj, 96, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "");
                }
                {
                    // LabelCh6
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.label_ch6 = obj;
                    lv_obj_set_pos(obj, 202, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "");
                }
            }
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
    objects.label_counter = 0;
    objects.start_button = 0;
    objects.button_label = 0;
    objects.counter_label = 0;
    objects.banner = 0;
    objects.label_ch1 = 0;
    objects.label_ch2 = 0;
    objects.label_ch3 = 0;
    objects.label_ch4 = 0;
    objects.label_ch5 = 0;
    objects.label_ch6 = 0;
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
        const char *new_val = get_var_counter();
        const char *cur_val = lv_label_get_text(objects.counter_label);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.counter_label;
            lv_label_set_text(objects.counter_label, new_val);
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
            lv_obj_set_style_bg_color(obj, lv_color_hex(0x8d9cf1), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
            {
                lv_obj_t *parent_obj = obj;
                {
                    // TabRadio
                    lv_obj_t *obj = lv_tabview_add_tab(parent_obj, "Radio");
                    objects.tab_radio = obj;
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                    {
                        lv_obj_t *parent_obj = obj;
                        {
                            lv_obj_t *obj = lv_switch_create(parent_obj);
                            lv_obj_set_pos(obj, 0, 0);
                            lv_obj_set_size(obj, 51, 25);
                        }
                    }
                }
                {
                    // TabSwitch
                    lv_obj_t *obj = lv_tabview_add_tab(parent_obj, "Switch");
                    objects.tab_switch = obj;
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                    {
                        lv_obj_t *parent_obj = obj;
                        {
                            // MenuBackLabel
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.menu_back_label = obj;
                            lv_obj_set_pos(obj, -243, -158);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Channel 1 ");
                        }
                        {
                            // MenuBackLabel_3
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.menu_back_label_3 = obj;
                            lv_obj_set_pos(obj, -243, 85);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Channel 4");
                        }
                        {
                            // MenuBackLabel_1
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.menu_back_label_1 = obj;
                            lv_obj_set_pos(obj, -243, -77);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Channel 2");
                        }
                        {
                            // MenuBackLabel_2
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.menu_back_label_2 = obj;
                            lv_obj_set_pos(obj, -243, 4);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Channel 3");
                        }
                        {
                            // Channel1
                            lv_obj_t *obj = lv_dropdown_create(parent_obj);
                            objects.channel1 = obj;
                            lv_obj_set_pos(obj, 121, 13);
                            lv_obj_set_size(obj, 150, LV_SIZE_CONTENT);
                            lv_dropdown_set_options_static(obj, "Roulis\nTangage\nGaz\nLacet");
                            lv_obj_add_event_cb(obj, event_handler_cb_settings_channel1, LV_EVENT_ALL, 0);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                        {
                            // Channel1_1
                            lv_obj_t *obj = lv_dropdown_create(parent_obj);
                            objects.channel1_1 = obj;
                            lv_obj_set_pos(obj, 121, 94);
                            lv_obj_set_size(obj, 150, LV_SIZE_CONTENT);
                            lv_dropdown_set_options_static(obj, "Roulis\nTangage\nGaz\nLacet");
                            lv_obj_add_event_cb(obj, event_handler_cb_settings_channel1_1, LV_EVENT_ALL, 0);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                        {
                            // Channel1_2
                            lv_obj_t *obj = lv_dropdown_create(parent_obj);
                            objects.channel1_2 = obj;
                            lv_obj_set_pos(obj, 121, 175);
                            lv_obj_set_size(obj, 150, LV_SIZE_CONTENT);
                            lv_dropdown_set_options_static(obj, "Roulis\nTangage\nGaz\nLacet");
                            lv_obj_add_event_cb(obj, event_handler_cb_settings_channel1_2, LV_EVENT_ALL, 0);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                        {
                            // Channel1_3
                            lv_obj_t *obj = lv_dropdown_create(parent_obj);
                            objects.channel1_3 = obj;
                            lv_obj_set_pos(obj, 121, 256);
                            lv_obj_set_size(obj, 150, LV_SIZE_CONTENT);
                            lv_dropdown_set_options_static(obj, "Roulis\nTangage\nGaz\nLacet");
                            lv_obj_add_event_cb(obj, event_handler_cb_settings_channel1_3, LV_EVENT_ALL, 0);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                        {
                            // Sw-A
                            lv_obj_t *obj = lv_switch_create(parent_obj);
                            objects.sw_a = obj;
                            lv_obj_set_pos(obj, 520, 26);
                            lv_obj_set_size(obj, 50, 25);
                            lv_obj_add_event_cb(obj, event_handler_cb_settings_sw_a, LV_EVENT_ALL, 0);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0x42ed08), LV_PART_INDICATOR | LV_STATE_CHECKED);
                        }
                        {
                            // Sw-A_1
                            lv_obj_t *obj = lv_switch_create(parent_obj);
                            objects.sw_a_1 = obj;
                            lv_obj_set_pos(obj, 520, 188);
                            lv_obj_set_size(obj, 50, 22);
                            lv_obj_add_event_cb(obj, event_handler_cb_settings_sw_a_1, LV_EVENT_ALL, 0);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0x42ed08), LV_PART_INDICATOR | LV_STATE_CHECKED);
                        }
                        {
                            // Sw-A_2
                            lv_obj_t *obj = lv_switch_create(parent_obj);
                            objects.sw_a_2 = obj;
                            lv_obj_set_pos(obj, 520, 265);
                            lv_obj_set_size(obj, 50, 25);
                            lv_obj_add_event_cb(obj, event_handler_cb_settings_sw_a_2, LV_EVENT_ALL, 0);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0x42ed08), LV_PART_INDICATOR | LV_STATE_CHECKED);
                        }
                        {
                            // Sw1_1
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.sw1_1 = obj;
                            lv_obj_set_pos(obj, 400, 107);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_label_set_text_static(obj, "SW-2");
                        }
                        {
                            // SW-2
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.sw_2 = obj;
                            lv_obj_set_pos(obj, 400, 188);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_label_set_text_static(obj, "SW-3");
                        }
                        {
                            // Sw1
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.sw1 = obj;
                            lv_obj_set_pos(obj, 402, 26);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_label_set_text_static(obj, "SW-1");
                        }
                        {
                            // Sw-A_5
                            lv_obj_t *obj = lv_switch_create(parent_obj);
                            objects.sw_a_5 = obj;
                            lv_obj_set_pos(obj, 520, 107);
                            lv_obj_set_size(obj, 50, 25);
                            lv_obj_add_event_cb(obj, event_handler_cb_settings_sw_a_5, LV_EVENT_ALL, 0);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0x42ed08), LV_PART_INDICATOR | LV_STATE_CHECKED);
                        }
                        {
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            lv_obj_set_pos(obj, 398, 269);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_label_set_text_static(obj, "SW-4");
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
                            lv_obj_set_pos(obj, 0, 107);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text_static(obj, "Current Device  : ");
                        }
                        {
                            // DeviceList
                            lv_obj_t *obj = lv_dropdown_create(parent_obj);
                            objects.device_list = obj;
                            lv_obj_set_pos(obj, 155, 233);
                            lv_obj_set_size(obj, 150, LV_SIZE_CONTENT);
                            lv_dropdown_set_options_static(obj, "Option 1\nOption 2\nOption 3");
                            lv_dropdown_set_selected(obj, 0);
                        }
                        {
                            // LoadDeviceLabel
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.load_device_label = obj;
                            lv_obj_set_pos(obj, 0, 244);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_label_set_text_static(obj, "Load Device : ");
                        }
                        {
                            lv_obj_t *obj = lv_button_create(parent_obj);
                            objects.obj0 = obj;
                            lv_obj_set_pos(obj, 468, 169);
                            lv_obj_set_size(obj, 101, 50);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0xf5b169), LV_PART_MAIN | LV_STATE_DEFAULT);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    objects.obj1 = obj;
                                    lv_obj_set_pos(obj, 0, 0);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_text_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_label_set_text_static(obj, "New");
                                }
                            }
                        }
                        {
                            lv_obj_t *obj = lv_button_create(parent_obj);
                            objects.obj2 = obj;
                            lv_obj_set_pos(obj, 468, 265);
                            lv_obj_set_size(obj, 100, 50);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0xf5b169), LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_bg_opa(obj, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    objects.obj3 = obj;
                                    lv_obj_set_pos(obj, 0, 0);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_text_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_label_set_text_static(obj, "Delete");
                                }
                            }
                        }
                        {
                            // DeviceNameLabel
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.device_name_label = obj;
                            lv_obj_set_pos(obj, 171, 107);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_color(obj, lv_color_hex(0xfafafa), LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_border_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "");
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
    objects.tab_radio = 0;
    objects.tab_switch = 0;
    objects.menu_back_label = 0;
    objects.menu_back_label_3 = 0;
    objects.menu_back_label_1 = 0;
    objects.menu_back_label_2 = 0;
    objects.channel1 = 0;
    objects.channel1_1 = 0;
    objects.channel1_2 = 0;
    objects.channel1_3 = 0;
    objects.sw_a = 0;
    objects.sw_a_1 = 0;
    objects.sw_a_2 = 0;
    objects.sw1_1 = 0;
    objects.sw_2 = 0;
    objects.sw1 = 0;
    objects.sw_a_5 = 0;
    objects.device = 0;
    objects.device_list = 0;
    objects.load_device_label = 0;
    objects.obj0 = 0;
    objects.obj1 = 0;
    objects.obj2 = 0;
    objects.obj3 = 0;
    objects.device_name_label = 0;
    objects.menu_back = 0;
    objects.save = 0;
}

void tick_screen_settings() {
    {
        if (!(lv_obj_get_state(objects.channel1) & LV_STATE_EDITED)) {
            int32_t new_val = get_var_selection1();
            int32_t cur_val = lv_dropdown_get_selected(objects.channel1);
            if (new_val != cur_val) {
                tick_value_change_obj = objects.channel1;
                lv_dropdown_set_selected(objects.channel1, new_val);
                tick_value_change_obj = NULL;
            }
        }
    }
    {
        if (!(lv_obj_get_state(objects.channel1_1) & LV_STATE_EDITED)) {
            int32_t new_val = get_var_selection2();
            int32_t cur_val = lv_dropdown_get_selected(objects.channel1_1);
            if (new_val != cur_val) {
                tick_value_change_obj = objects.channel1_1;
                lv_dropdown_set_selected(objects.channel1_1, new_val);
                tick_value_change_obj = NULL;
            }
        }
    }
    {
        if (!(lv_obj_get_state(objects.channel1_2) & LV_STATE_EDITED)) {
            int32_t new_val = get_var_selection3();
            int32_t cur_val = lv_dropdown_get_selected(objects.channel1_2);
            if (new_val != cur_val) {
                tick_value_change_obj = objects.channel1_2;
                lv_dropdown_set_selected(objects.channel1_2, new_val);
                tick_value_change_obj = NULL;
            }
        }
    }
    {
        if (!(lv_obj_get_state(objects.channel1_3) & LV_STATE_EDITED)) {
            int32_t new_val = get_var_selection4();
            int32_t cur_val = lv_dropdown_get_selected(objects.channel1_3);
            if (new_val != cur_val) {
                tick_value_change_obj = objects.channel1_3;
                lv_dropdown_set_selected(objects.channel1_3, new_val);
                tick_value_change_obj = NULL;
            }
        }
    }
    {
        bool new_val = get_var_sw1();
        bool cur_val = lv_obj_has_state(objects.sw_a, LV_STATE_CHECKED);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.sw_a;
            if (new_val) {
                lv_obj_add_state(objects.sw_a, LV_STATE_CHECKED);
            } else {
                lv_obj_remove_state(objects.sw_a, LV_STATE_CHECKED);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = get_var_sw3();
        bool cur_val = lv_obj_has_state(objects.sw_a_1, LV_STATE_CHECKED);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.sw_a_1;
            if (new_val) {
                lv_obj_add_state(objects.sw_a_1, LV_STATE_CHECKED);
            } else {
                lv_obj_remove_state(objects.sw_a_1, LV_STATE_CHECKED);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = get_var_sw4();
        bool cur_val = lv_obj_has_state(objects.sw_a_2, LV_STATE_CHECKED);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.sw_a_2;
            if (new_val) {
                lv_obj_add_state(objects.sw_a_2, LV_STATE_CHECKED);
            } else {
                lv_obj_remove_state(objects.sw_a_2, LV_STATE_CHECKED);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = get_var_sw2();
        bool cur_val = lv_obj_has_state(objects.sw_a_5, LV_STATE_CHECKED);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.sw_a_5;
            if (new_val) {
                lv_obj_add_state(objects.sw_a_5, LV_STATE_CHECKED);
            } else {
                lv_obj_remove_state(objects.sw_a_5, LV_STATE_CHECKED);
            }
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

typedef void (*create_screen_func_t)();
create_screen_func_t create_screen_funcs[] = {
    create_screen_main,
    create_screen_settings,
    create_screen_splash_screen,
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
};
void tick_screen(int screen_index) {
    if (screen_index >= 0 && screen_index < 3) {
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
}