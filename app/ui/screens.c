#include <string.h>

#include "screens.h"
#include "images.h"
#include "fonts.h"
#include "actions.h"
#include "vars.h"
#include "styles.h"
#include "ui.h"

#include <string.h>

objects_t objects;
lv_obj_t *tick_value_change_obj;
uint32_t active_theme_index = 0;

static void event_handler_cb_settings_channel1(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target(e);
        if (tick_value_change_obj != ta) {
            int32_t value = lv_dropdown_get_selected(ta);
            set_var_selection1(value);
        }
    }
}

void create_screen_main() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.main = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 800, 480);
    {
        lv_obj_t *parent_obj = obj;
        {
            // Label_Counter
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.label_counter = obj;
            lv_obj_set_pos(obj, 317, 117);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text(obj, "Counter: ");
        }
        {
            // StartButton
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.start_button = obj;
            lv_obj_set_pos(obj, 350, 215);
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
                    lv_label_set_text(obj, "Start");
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
            lv_obj_set_pos(obj, 0, 0);
            lv_obj_set_size(obj, 800, 65);
            lv_obj_set_style_pad_left(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_top(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_right(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_bottom(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_opa(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_radius(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_grad_color(obj, lv_color_hex(0xffebe0e0), LV_PART_MAIN | LV_STATE_DEFAULT);
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
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xfff1c38d), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
    
    tick_screen_main();
}

void tick_screen_main() {
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
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff8d9cf1), LV_PART_MAIN | LV_STATE_DEFAULT);
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
                            lv_obj_t *obj = lv_checkbox_create(parent_obj);
                            lv_obj_set_pos(obj, 121, 1);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_checkbox_set_text(obj, "Aux1");
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
                            lv_label_set_text(obj, "Channel 1 ");
                        }
                        {
                            // MenuBackLabel_3
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.menu_back_label_3 = obj;
                            lv_obj_set_pos(obj, -243, 85);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "Channel 4");
                        }
                        {
                            // MenuBackLabel_1
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.menu_back_label_1 = obj;
                            lv_obj_set_pos(obj, -243, -77);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "Channel 2");
                        }
                        {
                            // MenuBackLabel_2
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.menu_back_label_2 = obj;
                            lv_obj_set_pos(obj, -243, 4);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "Channel 3");
                        }
                        {
                            // Channel1
                            lv_obj_t *obj = lv_dropdown_create(parent_obj);
                            objects.channel1 = obj;
                            lv_obj_set_pos(obj, 121, 13);
                            lv_obj_set_size(obj, 150, LV_SIZE_CONTENT);
                            lv_dropdown_set_options(obj, "Roulis\nTangage\nGaz\nLacet");
                            lv_obj_add_event_cb(obj, event_handler_cb_settings_channel1, LV_EVENT_ALL, 0);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                        {
                            // Channel1_1
                            lv_obj_t *obj = lv_dropdown_create(parent_obj);
                            objects.channel1_1 = obj;
                            lv_obj_set_pos(obj, 121, 94);
                            lv_obj_set_size(obj, 150, LV_SIZE_CONTENT);
                            lv_dropdown_set_options(obj, "Roulis\nTangage\nLacet\nGaz");
                            lv_dropdown_set_selected(obj, 1);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                        {
                            // Channel1_2
                            lv_obj_t *obj = lv_dropdown_create(parent_obj);
                            objects.channel1_2 = obj;
                            lv_obj_set_pos(obj, 121, 175);
                            lv_obj_set_size(obj, 150, LV_SIZE_CONTENT);
                            lv_dropdown_set_options(obj, "Roulis\nTangage\nLacet\nGaz");
                            lv_dropdown_set_selected(obj, 3);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                        {
                            // Channel1_3
                            lv_obj_t *obj = lv_dropdown_create(parent_obj);
                            objects.channel1_3 = obj;
                            lv_obj_set_pos(obj, 121, 256);
                            lv_obj_set_size(obj, 150, LV_SIZE_CONTENT);
                            lv_dropdown_set_options(obj, "Roulis\nTangage\nLacet\nGaz");
                            lv_dropdown_set_selected(obj, 2);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);
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
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff000000), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_image_src(obj, &img_back_icon, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
    
    tick_screen_settings();
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
}



typedef void (*tick_screen_func_t)();
tick_screen_func_t tick_screen_funcs[] = {
    tick_screen_main,
    tick_screen_settings,
};
void tick_screen(int screen_index) {
    tick_screen_funcs[screen_index]();
}
void tick_screen_by_id(enum ScreensEnum screenId) {
    tick_screen_funcs[screenId - 1]();
}

void create_screens() {
    lv_disp_t *dispp = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    
    create_screen_main();
    create_screen_settings();
}
