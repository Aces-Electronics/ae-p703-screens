// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.4
// Project name: ae_aux_front

#include "../ui.h"

void ui_splashScreen_screen_init(void)
{
ui_splashScreen = lv_obj_create(NULL);
lv_obj_clear_flag( ui_splashScreen, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_img_src( ui_splashScreen, &ui_img_ae_red_290_nobg_png, LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_img_recolor(ui_splashScreen, lv_color_hex(0xFFFFFF), LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_img_recolor_opa(ui_splashScreen, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_add_event_cb(ui_splashScreen, ui_event_splashScreen, LV_EVENT_ALL, NULL);

}
