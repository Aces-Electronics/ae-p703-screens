// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.6
// Project name: ae_aux_rear_basic

#include "../ui.h"

void ui_factoryResetScreen_screen_init(void)
{
ui_factoryResetScreen = lv_obj_create(NULL);
lv_obj_clear_flag( ui_factoryResetScreen, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM );    /// Flags
lv_obj_set_scroll_dir(ui_factoryResetScreen, LV_DIR_HOR);

ui_factoryResetText = lv_label_create(ui_factoryResetScreen);
lv_obj_set_width( ui_factoryResetText, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_factoryResetText, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_factoryResetText, 1 );
lv_obj_set_y( ui_factoryResetText, -64 );
lv_obj_set_align( ui_factoryResetText, LV_ALIGN_CENTER );
lv_label_set_text(ui_factoryResetText,"Need to factory reset?\nHold the RESET \nbutton");
lv_obj_clear_flag( ui_factoryResetText, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_align(ui_factoryResetText, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_factoryResetText, &lv_font_montserrat_42, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_factoryResetButton = lv_btn_create(ui_factoryResetScreen);
lv_obj_set_width( ui_factoryResetButton, 157);
lv_obj_set_height( ui_factoryResetButton, 77);
lv_obj_set_x( ui_factoryResetButton, 98 );
lv_obj_set_y( ui_factoryResetButton, 76 );
lv_obj_set_align( ui_factoryResetButton, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_factoryResetButton, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_bg_color(ui_factoryResetButton, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_factoryResetButton, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_resetLabel = lv_label_create(ui_factoryResetScreen);
lv_obj_set_width( ui_resetLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_resetLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_resetLabel, 99 );
lv_obj_set_y( ui_resetLabel, 76 );
lv_obj_set_align( ui_resetLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_resetLabel,"RESET");
lv_obj_add_flag( ui_resetLabel, LV_OBJ_FLAG_CLICKABLE );   /// Flags
lv_obj_clear_flag( ui_resetLabel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_scrollbar_mode(ui_resetLabel, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_style_text_font(ui_resetLabel, &lv_font_montserrat_42, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Button1 = lv_btn_create(ui_factoryResetScreen);
lv_obj_set_width( ui_Button1, 157);
lv_obj_set_height( ui_Button1, 77);
lv_obj_set_x( ui_Button1, -96 );
lv_obj_set_y( ui_Button1, 76 );
lv_obj_set_align( ui_Button1, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_Button1, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_bg_color(ui_Button1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Button1, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label1 = lv_label_create(ui_factoryResetScreen);
lv_obj_set_width( ui_Label1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label1, -96 );
lv_obj_set_y( ui_Label1, 76 );
lv_obj_set_align( ui_Label1, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label1,"BACK");
lv_obj_add_flag( ui_Label1, LV_OBJ_FLAG_CLICKABLE );   /// Flags
lv_obj_clear_flag( ui_Label1, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_color(ui_Label1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label1, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label1, &lv_font_montserrat_42, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_add_event_cb(ui_resetLabel, ui_event_resetLabel, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Label1, ui_event_Label1, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_factoryResetScreen, ui_event_factoryResetScreen, LV_EVENT_ALL, NULL);

}
