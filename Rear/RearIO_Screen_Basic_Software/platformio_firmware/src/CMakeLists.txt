SET(SOURCES screens/ui_splashScreen.c
    screens/ui_batteryStateScreen.c
    screens/ui_accessoryStateScreen.c
    screens/ui_settingsStateScreen.c
    screens/ui_Factory_Reset.c
    ui.c
    ui_helpers.c
    images/ui_img_ae_red_290_nobg_png.c)

add_library(ui ${SOURCES})
