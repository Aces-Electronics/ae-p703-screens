// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.6
// Project name: ae_aux_rear_basic

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////

// SCREEN: ui_splashScreen
void ui_splashScreen_screen_init(void);
void ui_event_splashScreen( lv_event_t * e);
lv_obj_t *ui_splashScreen;

// SCREEN: ui_batteryStateScreen
void ui_batteryStateScreen_screen_init(void);
void ui_event_batteryStateScreen( lv_event_t * e);
lv_obj_t *ui_batteryStateScreen;
lv_obj_t *ui_auxBattVoltageArc;
lv_obj_t *ui_auxBattPercentageLabel;
lv_obj_t *ui_warningLabel;
lv_obj_t *ui_auxBatteryStateLegend;
lv_obj_t *ui_socLabel;
lv_obj_t *ui_auxBattVoltageLabel;
lv_obj_t *ui_auxState;
void ui_event_hp1Label( lv_event_t * e);
lv_obj_t *ui_hp1Label;
void ui_event_hp2Label( lv_event_t * e);
lv_obj_t *ui_hp2Label;
void ui_event_lp1Label( lv_event_t * e);
lv_obj_t *ui_lp1Label;
lv_obj_t *ui_auxBatteryStateLegend1;
lv_obj_t *ui_auxState1;

// SCREEN: ui_accessoryStateScreen
void ui_accessoryStateScreen_screen_init(void);
void ui_event_accessoryStateScreen( lv_event_t * e);
lv_obj_t *ui_accessoryStateScreen;
void ui_event_io1( lv_event_t * e);
lv_obj_t *ui_io1;
void ui_event_io2( lv_event_t * e);
lv_obj_t *ui_io2;
void ui_event_io3( lv_event_t * e);
lv_obj_t *ui_io3;
lv_obj_t *ui_ioLabel1;
lv_obj_t *ui_ioLabel2;
lv_obj_t *ui_ioLabel3;
lv_obj_t *ui_switchesLabel;

// SCREEN: ui_settingsStateScreen
void ui_settingsStateScreen_screen_init(void);
void ui_event_settingsStateScreen( lv_event_t * e);
lv_obj_t *ui_settingsStateScreen;
lv_obj_t *ui_settingsLabel;
lv_obj_t *ui_brightnessLabel;
void ui_event_brightnessBar( lv_event_t * e);
lv_obj_t *ui_brightnessBar;
lv_obj_t *ui_hp1DisplayLabel;
lv_obj_t *ui_hp1DisplayLabel1;
lv_obj_t *ui_hp1DisplayLabel2;
void ui_event_hp1TextArea( lv_event_t * e);
lv_obj_t *ui_hp1TextArea;
void ui_event_hp2TextArea( lv_event_t * e);
lv_obj_t *ui_hp2TextArea;
void ui_event_lp1TextArea( lv_event_t * e);
lv_obj_t *ui_lp1TextArea;
void ui_event_settingsKeyboard( lv_event_t * e);
lv_obj_t *ui_settingsKeyboard;
lv_obj_t *ui_resetButton;
void ui_event_resetPanel( lv_event_t * e);
lv_obj_t *ui_resetPanel;

// SCREEN: ui_factoryResetScreen
void ui_factoryResetScreen_screen_init(void);
void ui_event_factoryResetScreen( lv_event_t * e);
lv_obj_t *ui_factoryResetScreen;
lv_obj_t *ui_factoryResetText;
lv_obj_t *ui_factoryResetButton;
void ui_event_resetLabel( lv_event_t * e);
lv_obj_t *ui_resetLabel;
lv_obj_t *ui_Button1;
void ui_event_Label1( lv_event_t * e);
lv_obj_t *ui_Label1;
lv_obj_t *ui____initial_actions0;

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=0
    #error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_splashScreen( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_SCREEN_LOADED) {
      _ui_screen_change( ui_batteryStateScreen, LV_SCR_LOAD_ANIM_NONE, 500, 1000);
}
}
void ui_event_batteryStateScreen( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_TOP  ) {
lv_indev_wait_release(lv_indev_get_act());
      _ui_screen_change( ui_settingsStateScreen, LV_SCR_LOAD_ANIM_MOVE_TOP, 500, 0);
}
if ( event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_BOTTOM  ) {
lv_indev_wait_release(lv_indev_get_act());
      _ui_screen_change( ui_accessoryStateScreen, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 500, 0);
}
}
void ui_event_hp1Label( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      hp1ToggleFunction( e );
}
}
void ui_event_hp2Label( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      hp2ToggleFunction( e );
}
}
void ui_event_lp1Label( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      lp1ToggleFunction( e );
}
}
void ui_event_accessoryStateScreen( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_TOP  ) {
lv_indev_wait_release(lv_indev_get_act());
      _ui_screen_change( ui_batteryStateScreen, LV_SCR_LOAD_ANIM_MOVE_TOP, 500, 0);
}
if ( event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_BOTTOM  ) {
lv_indev_wait_release(lv_indev_get_act());
      _ui_screen_change( ui_settingsStateScreen, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 500, 0);
}
}
void ui_event_io1( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      hp1ToggleFunction( e );
}
}
void ui_event_io2( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      hp2ToggleFunction( e );
}
}
void ui_event_io3( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      lp1ToggleFunction( e );
}
}
void ui_event_settingsStateScreen( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_TOP  ) {
lv_indev_wait_release(lv_indev_get_act());
      _ui_screen_change( ui_accessoryStateScreen, LV_SCR_LOAD_ANIM_MOVE_TOP, 500, 0);
}
if ( event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_BOTTOM  ) {
lv_indev_wait_release(lv_indev_get_act());
      _ui_screen_change( ui_batteryStateScreen, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 500, 0);
}
}
void ui_event_brightnessBar( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_VALUE_CHANGED) {
      set_screen_brightness( e );
}
}
void ui_event_settingsKeyboard( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
}
void ui_event_resetPanel( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      _ui_screen_change( ui_factoryResetScreen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0);
}
}
void ui_event_factoryResetScreen( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT  ) {
lv_indev_wait_release(lv_indev_get_act());
      _ui_screen_change( ui_batteryStateScreen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0);
}
if ( event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT  ) {
lv_indev_wait_release(lv_indev_get_act());
      _ui_screen_change( ui_settingsStateScreen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 500, 0);
}
}
void ui_event_resetLabel( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_LONG_PRESSED) {
      factoryReset( e );
}
}
void ui_event_Label1( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      _ui_screen_change( ui_settingsStateScreen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 500, 0);
}
}

///////////////////// SCREENS ////////////////////

void ui_init( void )
{
lv_disp_t *dispp = lv_disp_get_default();
lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
lv_disp_set_theme(dispp, theme);
ui_splashScreen_screen_init();
ui_batteryStateScreen_screen_init();
ui_accessoryStateScreen_screen_init();
ui_settingsStateScreen_screen_init();
ui_factoryResetScreen_screen_init();
ui____initial_actions0 = lv_obj_create(NULL);
lv_disp_load_scr( ui_splashScreen);
}
