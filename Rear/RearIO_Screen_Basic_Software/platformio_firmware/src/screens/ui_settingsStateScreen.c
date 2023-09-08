// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.4
// Project name: ae_aux

#include "../ui.h"

void ui_settingsStateScreen_screen_init(void)
{
ui_settingsStateScreen = lv_obj_create(NULL);
lv_obj_clear_flag( ui_settingsStateScreen, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_settingsLabel = lv_label_create(ui_settingsStateScreen);
lv_obj_set_width( ui_settingsLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_settingsLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_settingsLabel, LV_ALIGN_TOP_MID );
lv_label_set_text(ui_settingsLabel,"Settings");
lv_obj_set_style_text_font(ui_settingsLabel, &lv_font_montserrat_42, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_brightnessLabel = lv_label_create(ui_settingsStateScreen);
lv_obj_set_width( ui_brightnessLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_brightnessLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_brightnessLabel, -145 );
lv_obj_set_y( ui_brightnessLabel, 267 );
lv_obj_set_align( ui_brightnessLabel, LV_ALIGN_TOP_MID );
lv_label_set_text(ui_brightnessLabel,"Brightness");
lv_obj_set_style_text_font(ui_brightnessLabel, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_brightnessBar = lv_slider_create(ui_settingsStateScreen);
lv_slider_set_value( ui_brightnessBar, 100, LV_ANIM_OFF);
if (lv_slider_get_mode(ui_brightnessBar)==LV_SLIDER_MODE_RANGE ) lv_slider_set_left_value( ui_brightnessBar, 0, LV_ANIM_OFF);
lv_obj_set_width( ui_brightnessBar, 237);
lv_obj_set_height( ui_brightnessBar, 15);
lv_obj_set_x( ui_brightnessBar, 80 );
lv_obj_set_y( ui_brightnessBar, 124 );
lv_obj_set_align( ui_brightnessBar, LV_ALIGN_CENTER );
lv_obj_set_style_bg_color(ui_brightnessBar, lv_color_hex(0x666666), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_brightnessBar, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_hp1DisplayLabel = lv_label_create(ui_settingsStateScreen);
lv_obj_set_width( ui_hp1DisplayLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_hp1DisplayLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_hp1DisplayLabel, -163 );
lv_obj_set_y( ui_hp1DisplayLabel, 57 );
lv_obj_set_align( ui_hp1DisplayLabel, LV_ALIGN_TOP_MID );
lv_label_set_text(ui_hp1DisplayLabel,"HP1 Text");
lv_obj_set_style_text_font(ui_hp1DisplayLabel, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_hp1DisplayLabel1 = lv_label_create(ui_settingsStateScreen);
lv_obj_set_width( ui_hp1DisplayLabel1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_hp1DisplayLabel1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_hp1DisplayLabel1, -160 );
lv_obj_set_y( ui_hp1DisplayLabel1, 108 );
lv_obj_set_align( ui_hp1DisplayLabel1, LV_ALIGN_TOP_MID );
lv_label_set_text(ui_hp1DisplayLabel1,"HP2 Text");
lv_obj_set_style_text_font(ui_hp1DisplayLabel1, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_hp1DisplayLabel2 = lv_label_create(ui_settingsStateScreen);
lv_obj_set_width( ui_hp1DisplayLabel2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_hp1DisplayLabel2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_hp1DisplayLabel2, -166 );
lv_obj_set_y( ui_hp1DisplayLabel2, 162 );
lv_obj_set_align( ui_hp1DisplayLabel2, LV_ALIGN_TOP_MID );
lv_label_set_text(ui_hp1DisplayLabel2,"LP1 Text");
lv_obj_set_style_text_font(ui_hp1DisplayLabel2, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_hp1DisplayLabel3 = lv_label_create(ui_settingsStateScreen);
lv_obj_set_width( ui_hp1DisplayLabel3, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_hp1DisplayLabel3, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_hp1DisplayLabel3, -162 );
lv_obj_set_y( ui_hp1DisplayLabel3, 215 );
lv_obj_set_align( ui_hp1DisplayLabel3, LV_ALIGN_TOP_MID );
lv_label_set_text(ui_hp1DisplayLabel3,"LP2 Text");
lv_obj_set_style_text_font(ui_hp1DisplayLabel3, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_hp1TextArea = lv_textarea_create(ui_settingsStateScreen);
lv_obj_set_width( ui_hp1TextArea, 246);
lv_obj_set_height( ui_hp1TextArea, LV_SIZE_CONTENT);   /// 25
lv_obj_set_x( ui_hp1TextArea, 87 );
lv_obj_set_y( ui_hp1TextArea, -85 );
lv_obj_set_align( ui_hp1TextArea, LV_ALIGN_CENTER );
lv_textarea_set_max_length(ui_hp1TextArea,6);
lv_textarea_set_placeholder_text(ui_hp1TextArea,"HP1");
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
lv_obj_set_height( ui_hp2TextArea, LV_SIZE_CONTENT);   /// 25
lv_obj_set_x( ui_hp2TextArea, 87 );
lv_obj_set_y( ui_hp2TextArea, -33 );
lv_obj_set_align( ui_hp2TextArea, LV_ALIGN_CENTER );
lv_textarea_set_max_length(ui_hp2TextArea,6);
lv_textarea_set_placeholder_text(ui_hp2TextArea,"HP2");
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
lv_obj_set_height( ui_lp1TextArea, LV_SIZE_CONTENT);   /// 25
lv_obj_set_x( ui_lp1TextArea, 87 );
lv_obj_set_y( ui_lp1TextArea, 19 );
lv_obj_set_align( ui_lp1TextArea, LV_ALIGN_CENTER );
lv_textarea_set_max_length(ui_lp1TextArea,6);
lv_textarea_set_placeholder_text(ui_lp1TextArea,"LP1");
lv_textarea_set_one_line(ui_lp1TextArea,true);
lv_obj_clear_flag( ui_lp1TextArea, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_color(ui_lp1TextArea, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_lp1TextArea, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_lp1TextArea, &lv_font_montserrat_20, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_text_align(ui_lp1TextArea, LV_TEXT_ALIGN_AUTO, LV_PART_CURSOR| LV_STATE_DEFAULT);

lv_obj_set_style_text_color(ui_lp1TextArea, lv_color_hex(0x808080), LV_PART_TEXTAREA_PLACEHOLDER | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_lp1TextArea, 255, LV_PART_TEXTAREA_PLACEHOLDER| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_lp1TextArea, &lv_font_montserrat_20, LV_PART_TEXTAREA_PLACEHOLDER| LV_STATE_DEFAULT);

ui_lp2TextArea = lv_textarea_create(ui_settingsStateScreen);
lv_obj_set_width( ui_lp2TextArea, 246);
lv_obj_set_height( ui_lp2TextArea, LV_SIZE_CONTENT);   /// 25
lv_obj_set_x( ui_lp2TextArea, 87 );
lv_obj_set_y( ui_lp2TextArea, 71 );
lv_obj_set_align( ui_lp2TextArea, LV_ALIGN_CENTER );
lv_textarea_set_max_length(ui_lp2TextArea,6);
lv_textarea_set_placeholder_text(ui_lp2TextArea,"LP2");
lv_textarea_set_one_line(ui_lp2TextArea,true);
lv_obj_clear_flag( ui_lp2TextArea, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_color(ui_lp2TextArea, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_lp2TextArea, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_lp2TextArea, &lv_font_montserrat_20, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_text_align(ui_lp2TextArea, LV_TEXT_ALIGN_AUTO, LV_PART_CURSOR| LV_STATE_DEFAULT);

lv_obj_set_style_text_color(ui_lp2TextArea, lv_color_hex(0x808080), LV_PART_TEXTAREA_PLACEHOLDER | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_lp2TextArea, 255, LV_PART_TEXTAREA_PLACEHOLDER| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_lp2TextArea, &lv_font_montserrat_20, LV_PART_TEXTAREA_PLACEHOLDER| LV_STATE_DEFAULT);

ui_settingsKeyboard = lv_keyboard_create(ui_settingsStateScreen);
lv_obj_set_width( ui_settingsKeyboard, 454);
lv_obj_set_height( ui_settingsKeyboard, 179);
lv_obj_set_x( ui_settingsKeyboard, 1 );
lv_obj_set_y( ui_settingsKeyboard, 57 );
lv_obj_set_align( ui_settingsKeyboard, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_settingsKeyboard, LV_OBJ_FLAG_HIDDEN );   /// Flags

lv_obj_add_event_cb(ui_brightnessBar, ui_event_brightnessBar, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_hp1TextArea, ui_event_hp1TextArea, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_hp2TextArea, ui_event_hp2TextArea, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_lp1TextArea, ui_event_lp1TextArea, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_lp2TextArea, ui_event_lp2TextArea, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_settingsStateScreen, ui_event_settingsStateScreen, LV_EVENT_ALL, NULL);

}
