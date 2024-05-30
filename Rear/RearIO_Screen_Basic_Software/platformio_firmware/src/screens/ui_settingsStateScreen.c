// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.6
// Project name: ae_aux_rear_basic

#include "../ui.h"

void ui_settingsStateScreen_screen_init(void)
{
ui_settingsStateScreen = lv_obj_create(NULL);
lv_obj_clear_flag( ui_settingsStateScreen, LV_OBJ_FLAG_CLICKABLE );    /// Flags
lv_obj_set_scroll_dir(ui_settingsStateScreen, LV_DIR_HOR);

ui_settingsLabel = lv_label_create(ui_settingsStateScreen);
lv_obj_set_width( ui_settingsLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_settingsLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_settingsLabel, LV_ALIGN_TOP_MID );
lv_label_set_text(ui_settingsLabel,"SETTINGS");
lv_obj_clear_flag( ui_settingsLabel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_decor(ui_settingsLabel, LV_TEXT_DECOR_UNDERLINE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_settingsLabel, &lv_font_montserrat_40, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_brightnessLabel = lv_label_create(ui_settingsStateScreen);
lv_obj_set_width( ui_brightnessLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_brightnessLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_brightnessLabel, -138 );
lv_obj_set_y( ui_brightnessLabel, 64 );
lv_obj_set_align( ui_brightnessLabel, LV_ALIGN_TOP_MID );
lv_label_set_text(ui_brightnessLabel,"Brightness");
lv_obj_set_style_text_font(ui_brightnessLabel, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_brightnessBar = lv_slider_create(ui_settingsStateScreen);
lv_slider_set_value( ui_brightnessBar, 100, LV_ANIM_OFF);
if (lv_slider_get_mode(ui_brightnessBar)==LV_SLIDER_MODE_RANGE ) lv_slider_set_left_value( ui_brightnessBar, 0, LV_ANIM_OFF);
lv_obj_set_width( ui_brightnessBar, 237);
lv_obj_set_height( ui_brightnessBar, 15);
lv_obj_set_x( ui_brightnessBar, 81 );
lv_obj_set_y( ui_brightnessBar, -77 );
lv_obj_set_align( ui_brightnessBar, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_brightnessBar, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE );    /// Flags

lv_obj_set_style_bg_color(ui_brightnessBar, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_brightnessBar, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_brightnessBar, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_brightnessBar, 255, LV_PART_KNOB| LV_STATE_DEFAULT);

ui_hp1DisplayLabel = lv_label_create(ui_settingsStateScreen);
lv_obj_set_width( ui_hp1DisplayLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_hp1DisplayLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_hp1DisplayLabel, -138 );
lv_obj_set_y( ui_hp1DisplayLabel, 127 );
lv_obj_set_align( ui_hp1DisplayLabel, LV_ALIGN_TOP_MID );
lv_label_set_long_mode(ui_hp1DisplayLabel,LV_LABEL_LONG_SCROLL);
lv_label_set_text(ui_hp1DisplayLabel,"OP1 Name");
lv_obj_clear_flag( ui_hp1DisplayLabel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_font(ui_hp1DisplayLabel, &lv_font_montserrat_32, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_hp1DisplayLabel1 = lv_label_create(ui_settingsStateScreen);
lv_obj_set_width( ui_hp1DisplayLabel1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_hp1DisplayLabel1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_hp1DisplayLabel1, -138 );
lv_obj_set_y( ui_hp1DisplayLabel1, 195 );
lv_obj_set_align( ui_hp1DisplayLabel1, LV_ALIGN_TOP_MID );
lv_label_set_long_mode(ui_hp1DisplayLabel1,LV_LABEL_LONG_SCROLL);
lv_label_set_text(ui_hp1DisplayLabel1,"OP2 Name");
lv_obj_clear_flag( ui_hp1DisplayLabel1, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_font(ui_hp1DisplayLabel1, &lv_font_montserrat_32, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_hp1DisplayLabel2 = lv_label_create(ui_settingsStateScreen);
lv_obj_set_width( ui_hp1DisplayLabel2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_hp1DisplayLabel2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_hp1DisplayLabel2, -138 );
lv_obj_set_y( ui_hp1DisplayLabel2, 259 );
lv_obj_set_align( ui_hp1DisplayLabel2, LV_ALIGN_TOP_MID );
lv_label_set_text(ui_hp1DisplayLabel2,"OP3 Name");
lv_obj_clear_flag( ui_hp1DisplayLabel2, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_font(ui_hp1DisplayLabel2, &lv_font_montserrat_32, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_hp1TextArea = lv_textarea_create(ui_settingsStateScreen);
lv_obj_set_width( ui_hp1TextArea, 246);
lv_obj_set_height( ui_hp1TextArea, LV_SIZE_CONTENT);   /// 30
lv_obj_set_x( ui_hp1TextArea, 87 );
lv_obj_set_y( ui_hp1TextArea, -13 );
lv_obj_set_align( ui_hp1TextArea, LV_ALIGN_CENTER );
lv_textarea_set_max_length(ui_hp1TextArea,6);
lv_textarea_set_placeholder_text(ui_hp1TextArea,"Change Me!");
lv_textarea_set_one_line(ui_hp1TextArea,true);
lv_obj_clear_flag( ui_hp1TextArea, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_color(ui_hp1TextArea, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_hp1TextArea, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_hp1TextArea, &lv_font_montserrat_20, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_text_color(ui_hp1TextArea, lv_color_hex(0x808080), LV_PART_TEXTAREA_PLACEHOLDER | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_hp1TextArea, 255, LV_PART_TEXTAREA_PLACEHOLDER| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_hp1TextArea, &lv_font_montserrat_20, LV_PART_TEXTAREA_PLACEHOLDER| LV_STATE_DEFAULT);

ui_hp2TextArea = lv_textarea_create(ui_settingsStateScreen);
lv_obj_set_width( ui_hp2TextArea, 246);
lv_obj_set_height( ui_hp2TextArea, LV_SIZE_CONTENT);   /// 30
lv_obj_set_x( ui_hp2TextArea, 87 );
lv_obj_set_y( ui_hp2TextArea, 53 );
lv_obj_set_align( ui_hp2TextArea, LV_ALIGN_CENTER );
lv_textarea_set_max_length(ui_hp2TextArea,6);
lv_textarea_set_placeholder_text(ui_hp2TextArea,"Change Me!");
lv_textarea_set_one_line(ui_hp2TextArea,true);
lv_obj_clear_flag( ui_hp2TextArea, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_color(ui_hp2TextArea, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_hp2TextArea, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_hp2TextArea, &lv_font_montserrat_20, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_text_align(ui_hp2TextArea, LV_TEXT_ALIGN_AUTO, LV_PART_CURSOR| LV_STATE_DEFAULT);

lv_obj_set_style_text_color(ui_hp2TextArea, lv_color_hex(0x808080), LV_PART_TEXTAREA_PLACEHOLDER | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_hp2TextArea, 255, LV_PART_TEXTAREA_PLACEHOLDER| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_hp2TextArea, &lv_font_montserrat_20, LV_PART_TEXTAREA_PLACEHOLDER| LV_STATE_DEFAULT);

ui_lp1TextArea = lv_textarea_create(ui_settingsStateScreen);
lv_obj_set_width( ui_lp1TextArea, 246);
lv_obj_set_height( ui_lp1TextArea, LV_SIZE_CONTENT);   /// 30
lv_obj_set_x( ui_lp1TextArea, 87 );
lv_obj_set_y( ui_lp1TextArea, 118 );
lv_obj_set_align( ui_lp1TextArea, LV_ALIGN_CENTER );
lv_textarea_set_max_length(ui_lp1TextArea,6);
lv_textarea_set_placeholder_text(ui_lp1TextArea,"Change Me!");
lv_textarea_set_one_line(ui_lp1TextArea,true);
lv_obj_clear_flag( ui_lp1TextArea, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_color(ui_lp1TextArea, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_lp1TextArea, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_lp1TextArea, &lv_font_montserrat_20, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_text_align(ui_lp1TextArea, LV_TEXT_ALIGN_AUTO, LV_PART_CURSOR| LV_STATE_DEFAULT);

lv_obj_set_style_text_color(ui_lp1TextArea, lv_color_hex(0x808080), LV_PART_TEXTAREA_PLACEHOLDER | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_lp1TextArea, 255, LV_PART_TEXTAREA_PLACEHOLDER| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_lp1TextArea, &lv_font_montserrat_20, LV_PART_TEXTAREA_PLACEHOLDER| LV_STATE_DEFAULT);

ui_settingsKeyboard = lv_keyboard_create(ui_settingsStateScreen);
lv_keyboard_set_mode(ui_settingsKeyboard,LV_KEYBOARD_MODE_TEXT_UPPER);
lv_obj_set_width( ui_settingsKeyboard, 460);
lv_obj_set_height( ui_settingsKeyboard, 179);
lv_obj_set_x( ui_settingsKeyboard, 0 );
lv_obj_set_y( ui_settingsKeyboard, 54 );
lv_obj_set_align( ui_settingsKeyboard, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_settingsKeyboard, LV_OBJ_FLAG_HIDDEN );   /// Flags

ui_resetButton = lv_label_create(ui_settingsStateScreen);
lv_obj_set_width( ui_resetButton, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_resetButton, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_resetButton, 200 );
lv_obj_set_y( ui_resetButton, -135 );
lv_obj_set_align( ui_resetButton, LV_ALIGN_CENTER );
lv_label_set_text(ui_resetButton,"Reset");
lv_obj_clear_flag( ui_resetButton, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_scrollbar_mode(ui_resetButton, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_style_text_color(ui_resetButton, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_resetButton, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_resetButton, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_resetButton, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_clip_corner(ui_resetButton, true, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_outline_color(ui_resetButton, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_outline_opa(ui_resetButton, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_outline_width(ui_resetButton, 2, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_outline_pad(ui_resetButton, 3, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_resetButton, 3, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_resetButton, 3, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_resetButton, 2, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_resetButton, 2, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_resetPanel = lv_btn_create(ui_settingsStateScreen);
lv_obj_set_width( ui_resetPanel, 100);
lv_obj_set_height( ui_resetPanel, 50);
lv_obj_set_x( ui_resetPanel, 189 );
lv_obj_set_y( ui_resetPanel, -134 );
lv_obj_set_align( ui_resetPanel, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_resetPanel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_bg_color(ui_resetPanel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_resetPanel, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_add_event_cb(ui_brightnessBar, ui_event_brightnessBar, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_hp1TextArea, ui_event_hp1TextArea, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_hp2TextArea, ui_event_hp2TextArea, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_lp1TextArea, ui_event_lp1TextArea, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_settingsKeyboard, ui_event_settingsKeyboard, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_resetPanel, ui_event_resetPanel, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_settingsStateScreen, ui_event_settingsStateScreen, LV_EVENT_ALL, NULL);

}
