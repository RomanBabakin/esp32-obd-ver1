SET(SOURCES screens/ui_Screen1.c
    screens/ui_Screen2.c
    screens/ui_Screen3.c
    screens/ui_Screen4.c
    screens/ui_Screen5.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    fonts/ui_font_Signal_200_bold.c
    fonts/ui_font_Signal_30_Normal.c
    fonts/ui_font_Signal_70_bold.c)

add_library(ui ${SOURCES})
