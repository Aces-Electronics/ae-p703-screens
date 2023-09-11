// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.4
// Project name: ae_aux_front

#include "../ui.h"

void ui_batteryStateScreen_screen_init(void)
{
ui_batteryStateScreen = lv_obj_create(NULL);
lv_obj_clear_flag( ui_batteryStateScreen, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_scrollbar_mode(ui_batteryStateScreen, LV_SCROLLBAR_MODE_OFF);

ui_auxBattVoltageArc = lv_arc_create(ui_batteryStateScreen);
lv_obj_set_width( ui_auxBattVoltageArc, 160);
lv_obj_set_height( ui_auxBattVoltageArc, 162);
lv_obj_set_x( ui_auxBattVoltageArc, -2 );
lv_obj_set_y( ui_auxBattVoltageArc, 1 );
lv_obj_set_align( ui_auxBattVoltageArc, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_auxBattVoltageArc, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_arc_set_value(ui_auxBattVoltageArc, 75);
lv_obj_set_style_arc_width(ui_auxBattVoltageArc, 10, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0x3B8CE8), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_arc_opa(ui_auxBattVoltageArc, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_arc_width(ui_auxBattVoltageArc, 10, LV_PART_INDICATOR| LV_STATE_DEFAULT);

lv_obj_set_style_radius(ui_auxBattVoltageArc, 30, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0x3B8CE8), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_auxBattVoltageArc, 255, LV_PART_KNOB| LV_STATE_DEFAULT);

ui_auxBattPercentageLabel = lv_label_create(ui_batteryStateScreen);
lv_obj_set_width( ui_auxBattPercentageLabel, LV_SIZE_CONTENT);  /// 100
lv_obj_set_height( ui_auxBattPercentageLabel, LV_SIZE_CONTENT);   /// 100
lv_obj_set_x( ui_auxBattPercentageLabel, -2 );
lv_obj_set_y( ui_auxBattPercentageLabel, -33 );
lv_obj_set_align( ui_auxBattPercentageLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_auxBattPercentageLabel,"75%");
lv_obj_clear_flag( ui_auxBattPercentageLabel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_scrollbar_mode(ui_auxBattPercentageLabel, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0x3B8CE8), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_auxBattPercentageLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_auxBattPercentageLabel, &lv_font_montserrat_24, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_warningLabel = lv_label_create(ui_batteryStateScreen);
lv_obj_set_width( ui_warningLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_warningLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_warningLabel, 0 );
lv_obj_set_y( ui_warningLabel, 141 );
lv_obj_set_align( ui_warningLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_warningLabel,"");
lv_obj_set_style_text_color(ui_warningLabel, lv_color_hex(0xFE0303), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_warningLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_warningLabel, &lv_font_montserrat_20, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_auxBatteryStateLegend = lv_label_create(ui_batteryStateScreen);
lv_obj_set_width( ui_auxBatteryStateLegend, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_auxBatteryStateLegend, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_auxBatteryStateLegend, 118 );
lv_obj_set_y( ui_auxBatteryStateLegend, -70 );
lv_obj_set_align( ui_auxBatteryStateLegend, LV_ALIGN_CENTER );
lv_label_set_text(ui_auxBatteryStateLegend,"Battery");
lv_obj_clear_flag( ui_auxBatteryStateLegend, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_align(ui_auxBatteryStateLegend, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_auxBatteryStateLegend, &lv_font_montserrat_20, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_auxBattVoltageLabel = lv_label_create(ui_batteryStateScreen);
lv_obj_set_width( ui_auxBattVoltageLabel, LV_SIZE_CONTENT);  /// 100
lv_obj_set_height( ui_auxBattVoltageLabel, LV_SIZE_CONTENT);   /// 100
lv_obj_set_x( ui_auxBattVoltageLabel, -2 );
lv_obj_set_y( ui_auxBattVoltageLabel, -2 );
lv_obj_set_align( ui_auxBattVoltageLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_auxBattVoltageLabel,"12.60V");
lv_obj_clear_flag( ui_auxBattVoltageLabel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_scrollbar_mode(ui_auxBattVoltageLabel, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0x3B8CE8), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_auxBattVoltageLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_auxBattVoltageLabel, &lv_font_montserrat_24, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_auxState = lv_label_create(ui_batteryStateScreen);
lv_obj_set_width( ui_auxState, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_auxState, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_auxState, -2 );
lv_obj_set_y( ui_auxState, 29 );
lv_obj_set_align( ui_auxState, LV_ALIGN_CENTER );
lv_label_set_text(ui_auxState,"Mode");
lv_obj_clear_flag( ui_auxState, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_align(ui_auxState, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_auxState, &lv_font_montserrat_24, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_hp1Label = lv_label_create(ui_batteryStateScreen);
lv_obj_set_width( ui_hp1Label, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_hp1Label, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_hp1Label, -124 );
lv_obj_set_y( ui_hp1Label, 74 );
lv_obj_set_align( ui_hp1Label, LV_ALIGN_CENTER );
lv_label_set_text(ui_hp1Label,"HP1");
lv_obj_add_flag( ui_hp1Label, LV_OBJ_FLAG_CLICKABLE );   /// Flags
lv_obj_clear_flag( ui_hp1Label, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_color(ui_hp1Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_hp1Label, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_hp1Label, &lv_font_montserrat_22, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_hp2Label = lv_label_create(ui_batteryStateScreen);
lv_obj_set_width( ui_hp2Label, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_hp2Label, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_hp2Label, -44 );
lv_obj_set_y( ui_hp2Label, 74 );
lv_obj_set_align( ui_hp2Label, LV_ALIGN_CENTER );
lv_label_set_text(ui_hp2Label,"HP2");
lv_obj_add_flag( ui_hp2Label, LV_OBJ_FLAG_CLICKABLE );   /// Flags
lv_obj_clear_flag( ui_hp2Label, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_color(ui_hp2Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_hp2Label, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_hp2Label, &lv_font_montserrat_22, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_lp1Label = lv_label_create(ui_batteryStateScreen);
lv_obj_set_width( ui_lp1Label, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_lp1Label, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_lp1Label, 40 );
lv_obj_set_y( ui_lp1Label, 74 );
lv_obj_set_align( ui_lp1Label, LV_ALIGN_CENTER );
lv_label_set_text(ui_lp1Label,"LP1");
lv_obj_add_flag( ui_lp1Label, LV_OBJ_FLAG_CLICKABLE );   /// Flags
lv_obj_clear_flag( ui_lp1Label, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_color(ui_lp1Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_lp1Label, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_lp1Label, &lv_font_montserrat_22, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_lp2Label = lv_label_create(ui_batteryStateScreen);
lv_obj_set_width( ui_lp2Label, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_lp2Label, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_lp2Label, 119 );
lv_obj_set_y( ui_lp2Label, 74 );
lv_obj_set_align( ui_lp2Label, LV_ALIGN_CENTER );
lv_label_set_text(ui_lp2Label,"LP2");
lv_obj_add_flag( ui_lp2Label, LV_OBJ_FLAG_CLICKABLE );   /// Flags
lv_obj_clear_flag( ui_lp2Label, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_color(ui_lp2Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_lp2Label, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_lp2Label, &lv_font_montserrat_22, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_auxBatteryStateLegend1 = lv_label_create(ui_batteryStateScreen);
lv_obj_set_width( ui_auxBatteryStateLegend1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_auxBatteryStateLegend1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_auxBatteryStateLegend1, -110 );
lv_obj_set_y( ui_auxBatteryStateLegend1, -70 );
lv_obj_set_align( ui_auxBatteryStateLegend1, LV_ALIGN_CENTER );
lv_label_set_text(ui_auxBatteryStateLegend1,"Auxiliary ");
lv_obj_clear_flag( ui_auxBatteryStateLegend1, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_align(ui_auxBatteryStateLegend1, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_auxBatteryStateLegend1, &lv_font_montserrat_20, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_add_event_cb(ui_hp1Label, ui_event_hp1Label, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_hp2Label, ui_event_hp2Label, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_lp1Label, ui_event_lp1Label, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_lp2Label, ui_event_lp2Label, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_batteryStateScreen, ui_event_batteryStateScreen, LV_EVENT_ALL, NULL);

}
