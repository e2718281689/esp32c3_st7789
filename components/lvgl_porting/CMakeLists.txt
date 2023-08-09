idf_component_register(SRCS "lv_tack.c" "lv_port_indev.c" "lv_port_disp.c" "lv_tack.c"
                  INCLUDE_DIRS "."
                  REQUIRES lvgl_8.3 lvgl_esp32_drivers spiffs Peripheral_Devices)