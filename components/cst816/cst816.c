#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "driver/i2c.h"
#include "cst816.h"


#define I2C_MASTER_RX_BUF_DISABLE 0             
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_SDA_IO 7
#define I2C_MASTER_SCL_IO 6
#define I2C_MASTER_FREQ_HZ 400000        
#define I2C_MASTER_NUM 0

#define CST816S_NUM_RST 8
#define CST816S_NUM_INT 10
#define CST816S_SENSOR_ADDR 0x15

static SemaphoreHandle_t CST816S_Update_Semap;

struct CST816S_data_struct {
  uint8_t gestureID; // Gesture ID
  uint8_t points;  // Number of touch points
  uint8_t event; // Event (0 = Down, 1 = Up, 2 = Contact)
  int x;
  int y;
  uint8_t version;
  uint8_t versionInfo[3];
}CST816S_data;


esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
static void IRAM_ATTR CST816S_intrHandler (void *arg)
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(CST816S_Update_Semap,xHigherPriorityTaskWoken);
}

void CST816_INT_IO_Init()
{
     gpio_config_t CST816S_IO = {
        .pin_bit_mask = 1ULL<<CST816S_NUM_INT,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE,
        .pull_up_en = 1
    };
    gpio_config(&CST816S_IO);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CST816S_NUM_INT, CST816S_intrHandler, (void*)18);

}
static esp_err_t CST816_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, CST816S_SENSOR_ADDR, &reg_addr, 1, data, len, 100 / portTICK_RATE_MS);
}
static esp_err_t CST816_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, CST816S_SENSOR_ADDR, write_buf, sizeof(write_buf), 100 / portTICK_RATE_MS);

    return ret;
}
static void CST816S_Update_task(void* arg)
{
    while(1)
    {
        if(xSemaphoreTake(CST816S_Update_Semap, 10) == pdTRUE)
        {
            uint8_t data_raw[8];
            CST816_register_read(0x01, data_raw, 6);
            CST816S_data.gestureID = data_raw[0];
            CST816S_data.points = data_raw[1];
            CST816S_data.event = data_raw[2] >> 6;
            CST816S_data.x = data_raw[3];
            CST816S_data.y = data_raw[5];
            printf("x=%d,y=%d\n",CST816S_data.x,CST816S_data.y);
        }
    }
}
void CST816S_begin()
{

    //struct CST816S_data_struct CST816S_data;

    gpio_reset_pin( I2C_MASTER_SDA_IO );
    gpio_reset_pin( I2C_MASTER_SCL_IO );
    gpio_reset_pin( CST816S_NUM_RST );
    gpio_reset_pin( CST816S_NUM_INT );

    i2c_master_init();
    CST816_INT_IO_Init();

    CST816S_Update_Semap=xSemaphoreCreateBinary();

    xTaskCreate(CST816S_Update_task,"CST816S_Update",2048,NULL,10,NULL);


    gpio_set_direction(CST816S_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(CST816S_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(CST816S_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(CST816S_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);


    CST816_register_read(0x15,&CST816S_data.version,1);
    CST816_register_read(0xA7,CST816S_data.versionInfo,3);

}