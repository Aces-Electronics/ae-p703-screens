// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.4
// Project name: ae_aux_front

#ifndef _AE_AUX_FRONT_UI_H
#define _AE_AUX_FRONT_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined __has_include
  #if __has_include("lvgl.h")
    #include "lvgl.h"
  #elif __has_include("lvgl/lvgl.h")
    #include "lvgl/lvgl.h"
  #else
    #include "lvgl.h"
  #endif
#else
  #include "lvgl.h"
#endif

#include "ui_helpers.h"
#include "ui_events.h"
// SCREEN: ui_splashScreen
void ui_splashScreen_screen_init(void);
void ui_event_splashScreen( lv_event_t * e);
extern lv_obj_t *ui_splashScreen;
// SCREEN: ui_batteryStateScreen
void ui_batteryStateScreen_screen_init(void);
void ui_event_batteryStateScreen( lv_event_t * e);
extern lv_obj_t *ui_batteryStateScreen;
extern lv_obj_t *ui_auxBattVoltageArc;
extern lv_obj_t *ui_auxBattPercentageLabel;
extern lv_obj_t *ui_warningLabel;
extern lv_obj_t *ui_auxBatteryStateLegend;
extern lv_obj_t *ui_auxBattVoltageLabel;
extern lv_obj_t *ui_auxState;
void ui_event_hp1Label( lv_event_t * e);
extern lv_obj_t *ui_hp1Label;
void ui_event_hp2Label( lv_event_t * e);
extern lv_obj_t *ui_hp2Label;
void ui_event_lp1Label( lv_event_t * e);
extern lv_obj_t *ui_lp1Label;
void ui_event_lp2Label( lv_event_t * e);
extern lv_obj_t *ui_lp2Label;
extern lv_obj_t *ui_auxBatteryStateLegend1;
// SCREEN: ui_accessoryStateScreen
void ui_accessoryStateScreen_screen_init(void);
void ui_event_accessoryStateScreen( lv_event_t * e);
extern lv_obj_t *ui_accessoryStateScreen;
void ui_event_io1( lv_event_t * e);
extern lv_obj_t *ui_io1;
void ui_event_io2( lv_event_t * e);
extern lv_obj_t *ui_io2;
void ui_event_io3( lv_event_t * e);
extern lv_obj_t *ui_io3;
void ui_event_io4( lv_event_t * e);
extern lv_obj_t *ui_io4;
extern lv_obj_t *ui_ioLabel1;
extern lv_obj_t *ui_ioLabel2;
extern lv_obj_t *ui_ioLabel3;
extern lv_obj_t *ui_ioLabel4;
extern lv_obj_t *ui____initial_actions0;

LV_IMG_DECLARE( ui_img_ae_red_290_nobg_png);   // assets/ae_red_290_nobg.png

void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif