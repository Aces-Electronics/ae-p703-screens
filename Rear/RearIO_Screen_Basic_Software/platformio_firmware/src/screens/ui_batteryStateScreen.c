// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.6
// Project name: ae_aux_rear_basic

#include "../ui.h"

void ui_batteryStateScreen_screen_init(void)
{
ui_batteryStateScreen = lv_obj_create(NULL);
lv_obj_clear_flag( ui_batteryStateScreen, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_scrollbar_mode(ui_batteryStateScreen, LV_SCROLLBAR_MODE_OFF);

ui_auxBattVoltageArc = lv_arc_create(ui_batteryStateScreen);
lv_obj_set_width( ui_auxBattVoltageArc, 300);
lv_obj_set_height( ui_auxBattVoltageArc, 296);
lv_obj_set_x( ui_auxBattVoltageArc, -1 );
lv_obj_set_y( ui_auxBattVoltageArc, 0 );
lv_obj_set_align( ui_auxBattVoltageArc, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_auxBattVoltageArc, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_arc_set_value(ui_auxBattVoltageArc, 75);
lv_obj_set_style_arc_width(ui_auxBattVoltageArc, 15, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_auxBattVoltageArc, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_arc_opa(ui_auxBattVoltageArc, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_arc_width(ui_auxBattVoltageArc, 15, LV_PART_INDICATOR| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_auxBattVoltageArc, 255, LV_PART_KNOB| LV_STATE_DEFAULT);

ui_auxBattPercentageLabel = lv_label_create(ui_batteryStateScreen);
lv_obj_set_width( ui_auxBattPercentageLabel, LV_SIZE_CONTENT);  /// 100
lv_obj_set_height( ui_auxBattPercentageLabel, LV_SIZE_CONTENT);   /// 100
lv_obj_set_x( ui_auxBattPercentageLabel, 2 );
lv_obj_set_y( ui_auxBattPercentageLabel, -59 );
lv_obj_set_align( ui_auxBattPercentageLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_auxBattPercentageLabel,"WAIT...");
lv_obj_clear_flag( ui_auxBattPercentageLabel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_scrollbar_mode(ui_auxBattPercentageLabel, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_auxBattPercentageLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_auxBattPercentageLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_auxBattPercentageLabel, &lv_font_montserrat_42, LV_PART_MAIN| LV_STATE_DEFAULT);

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
lv_obj_set_x( ui_auxBatteryStateLegend, 176 );
lv_obj_set_y( ui_auxBatteryStateLegend, -137 );
lv_obj_set_align( ui_auxBatteryStateLegend, LV_ALIGN_CENTER );
lv_label_set_text(ui_auxBatteryStateLegend,"Battery");
lv_obj_clear_flag( ui_auxBatteryStateLegend, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_align(ui_auxBatteryStateLegend, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_auxBatteryStateLegend, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_socLabel = lv_label_create(ui_batteryStateScreen);
lv_obj_set_width( ui_socLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_socLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_socLabel, -2 );
lv_obj_set_y( ui_socLabel, -93 );
lv_obj_set_align( ui_socLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_socLabel,"SOC:");
lv_obj_clear_flag( ui_socLabel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_font(ui_socLabel, &lv_font_montserrat_20, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_auxBattVoltageLabel = lv_label_create(ui_batteryStateScreen);
lv_obj_set_width( ui_auxBattVoltageLabel, LV_SIZE_CONTENT);  /// 100
lv_obj_set_height( ui_auxBattVoltageLabel, LV_SIZE_CONTENT);   /// 100
lv_obj_set_x( ui_auxBattVoltageLabel, -1 );
lv_obj_set_y( ui_auxBattVoltageLabel, 6 );
lv_obj_set_align( ui_auxBattVoltageLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_auxBattVoltageLabel,"No \nConnection");
lv_obj_clear_flag( ui_auxBattVoltageLabel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_scrollbar_mode(ui_auxBattVoltageLabel, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_auxBattVoltageLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_auxBattVoltageLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_auxBattVoltageLabel, &lv_font_montserrat_42, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_auxState = lv_label_create(ui_batteryStateScreen);
lv_obj_set_width( ui_auxState, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_auxState, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_auxState, -1 );
lv_obj_set_y( ui_auxState, 89 );
lv_obj_set_align( ui_auxState, LV_ALIGN_CENTER );
lv_label_set_text(ui_auxState,"MODE");
lv_obj_clear_flag( ui_auxState, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_align(ui_auxState, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_auxState, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_hp1Label = lv_label_create(ui_batteryStateScreen);
lv_obj_set_width( ui_hp1Label, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_hp1Label, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_hp1Label, -167 );
lv_obj_set_y( ui_hp1Label, 136 );
lv_obj_set_align( ui_hp1Label, LV_ALIGN_CENTER );
lv_label_set_long_mode(ui_hp1Label,LV_LABEL_LONG_SCROLL);
lv_label_set_text(ui_hp1Label,"OP1");
lv_obj_add_flag( ui_hp1Label, LV_OBJ_FLAG_CLICKABLE );   /// Flags
lv_obj_clear_flag( ui_hp1Label, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_scroll_dir(ui_hp1Label, LV_DIR_HOR);
lv_obj_set_style_text_color(ui_hp1Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_hp1Label, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_hp1Label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_hp1Label, &lv_font_montserrat_32, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_hp2Label = lv_label_create(ui_batteryStateScreen);
lv_obj_set_width( ui_hp2Label, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_hp2Label, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_hp2Label, -1 );
lv_obj_set_y( ui_hp2Label, 136 );
lv_obj_set_align( ui_hp2Label, LV_ALIGN_CENTER );
lv_label_set_long_mode(ui_hp2Label,LV_LABEL_LONG_SCROLL);
lv_label_set_text(ui_hp2Label,"OP2");
lv_obj_add_flag( ui_hp2Label, LV_OBJ_FLAG_CLICKABLE );   /// Flags
lv_obj_clear_flag( ui_hp2Label, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_color(ui_hp2Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_hp2Label, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_hp2Label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_hp2Label, &lv_font_montserrat_32, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_lp1Label = lv_label_create(ui_batteryStateScreen);
lv_obj_set_width( ui_lp1Label, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_lp1Label, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_lp1Label, 167 );
lv_obj_set_y( ui_lp1Label, 136 );
lv_obj_set_align( ui_lp1Label, LV_ALIGN_CENTER );
lv_label_set_long_mode(ui_lp1Label,LV_LABEL_LONG_SCROLL);
lv_label_set_text(ui_lp1Label,"OP3");
lv_obj_add_flag( ui_lp1Label, LV_OBJ_FLAG_CLICKABLE );   /// Flags
lv_obj_clear_flag( ui_lp1Label, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_color(ui_lp1Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_lp1Label, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_lp1Label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_lp1Label, &lv_font_montserrat_32, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_auxBatteryStateLegend1 = lv_label_create(ui_batteryStateScreen);
lv_obj_set_width( ui_auxBatteryStateLegend1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_auxBatteryStateLegend1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_auxBatteryStateLegend1, -163 );
lv_obj_set_y( ui_auxBatteryStateLegend1, -137 );
lv_obj_set_align( ui_auxBatteryStateLegend1, LV_ALIGN_CENTER );
lv_label_set_text(ui_auxBatteryStateLegend1,"Auxiliary ");
lv_obj_clear_flag( ui_auxBatteryStateLegend1, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_align(ui_auxBatteryStateLegend1, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_auxBatteryStateLegend1, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_auxState1 = lv_label_create(ui_batteryStateScreen);
lv_obj_set_width( ui_auxState1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_auxState1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_auxState1, -1 );
lv_obj_set_y( ui_auxState1, 61 );
lv_obj_set_align( ui_auxState1, LV_ALIGN_CENTER );
lv_label_set_text(ui_auxState1,"State:");
lv_obj_clear_flag( ui_auxState1, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_font(ui_auxState1, &lv_font_montserrat_20, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_add_event_cb(ui_hp1Label, ui_event_hp1Label, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_hp2Label, ui_event_hp2Label, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_lp1Label, ui_event_lp1Label, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_batteryStateScreen, ui_event_batteryStateScreen, LV_EVENT_ALL, NULL);

}
