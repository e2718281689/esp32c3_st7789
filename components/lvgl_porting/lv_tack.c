
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lvgl.h"
#include "lv_port_indev.h"
#include "lv_port_disp.h"
#include "esp_system.h"
#include "lv_tack.h"
#include "esp_spiffs.h"

#include "sht30.h"
#include "mpu6050.h"
static const char *TAG="lv_tack";
lv_obj_t* temp_lab ;
float temp_data=0;
float hum_data=0;
static void lv_tick_task(void *arg)
{
    (void)arg;
    lv_tick_inc(10);
}

void lv_spiffs_init()
{
        ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/lvgl",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
}
static void lv_task(void* arg)
{
    while(1)
    {
        lv_task_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
static void temp_task(void* arg)
{
    while(1)
    {
        i2c_master_sensor_sht30_read(I2C_MASTER_NUM, &temp_data, &hum_data);
        //MPU6050ReadTemp(&temp_data);
        lv_label_set_text_fmt(temp_lab, "temp==%0.3f", temp_data); 
        printf("%.01f\n",temp_data);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
static void anim_x_cb(void * var, int32_t v)
{
    lv_obj_set_x(var, v);
}
void lVGL_init()
{
    lv_spiffs_init();
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init(); 


    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"};
    esp_timer_handle_t periodic_timer;

    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 10 * 1000));

    xTaskCreate(lv_task,"lv_task",1024*10,NULL,10,NULL);


    temp_lab = lv_label_create(lv_scr_act());
    //lv_label_set_text_fmt(temp_lab, "%d", 28);  // 显示数字
    lv_obj_center(temp_lab);

    xTaskCreate(temp_task,"temp_task",1024*20,NULL,10,NULL);

    // lv_obj_t * img;
    // img = lv_img_create(lv_scr_act());
    // /* Assuming a File system is attached to letter 'A'
    //  * E.g. set LV_USE_FS_STDIO 'A' in lv_conf.h */
    // lv_img_set_src(img, "/lvgl/zzz1.jpg");
    // //lv_obj_set_pos(img,0,0);
    // lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);

    // lv_obj_t * xxx;
    // xxx=lv_label_create(lv_scr_act());
    // lv_label_set_text(xxx, "Hello world");
    // lv_obj_align(xxx, LV_ALIGN_CENTER, 0, 0);       

    // lv_anim_t a;
    // lv_anim_init(&a);

    // lv_anim_set_var(&a, xxx);
    // lv_anim_set_values(&a, -200, 200);
    // lv_anim_set_time(&a, 10000);
    // lv_anim_set_playback_delay(&a, 100);
    // lv_anim_set_playback_time(&a, 3000);
    // lv_anim_set_repeat_delay(&a, 500);
    // lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    // lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);


    // lv_anim_set_exec_cb(&a, anim_x_cb);
    // lv_anim_start(&a);

}