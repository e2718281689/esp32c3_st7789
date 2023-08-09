idf_component_register(SRCS "./mpu6050/mpu6050.c"  "./sht30/sht30.c" 
                  INCLUDE_DIRS "./mpu6050/" "./sht30/"
                  REQUIRES lvgl_8.3 lvgl_esp32_drivers spiffs
)