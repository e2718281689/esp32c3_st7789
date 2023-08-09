/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "lvgl.h"
#include "lv_port_indev.h"
#include "lv_port_disp.h"
#include "esp_spiffs.h"
#include "esp_log.h"
#include <dirent.h>
#include <sys/_stdint.h>
#include "lv_tack.h"
#include "mpu6050.h"
#include "wifi_comm.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "sht30.h"

static const char * TAG="main";

void app_main(void)
{
    ESP_LOGI(TAG, "thie main");


    vTaskDelay(pdMS_TO_TICKS(2000));

    uint8_t mac[6] = {0x12, 0x34, 0x56, 0x78, 0x90};
    esp_base_mac_addr_set(mac);

    //ESP_ERROR_CHECK(nvs_flash_erase()); //有的时候莫名其妙WiFi 、 mac地址都报废，用这个擦除nvs就好了
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    //wifi_init_sta();
    lVGL_init();
    Mpu6050_init();
    //i2c_sht30_task(NULL);

    
    // while(1)
    // {
    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }
}
