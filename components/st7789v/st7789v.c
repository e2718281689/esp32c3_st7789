#include "st7789v.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"


#define LCD_HOST    SPI2_HOST
#define PIN_NUM_MISO -1
#define PIN_NUM_MOSI 4
#define PIN_NUM_CLK  3
#define PIN_NUM_CS   2

#define PIN_NUM_DC   1
#define PIN_NUM_RST  5
#define PIN_NUM_BCKL 0

#define PARALLEL_LINES 16

spi_device_handle_t spi;

/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

void lcd_spi_init()
{
    esp_err_t ret;

    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=PARALLEL_LINES*320*2+8
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=10*1000*1000,           //Clock out at 10 MHz

        .mode=2,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
        //.flags=SPI_DEVICE_3WIRE
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO );
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}
void write_command(uint8_t cmd)
{
    lcd_cmd(spi,cmd);
}
void write_data(uint8_t date)
{
    lcd_data(spi,&date,1);
}
void lcd_init()
{

    gpio_reset_pin( PIN_NUM_MOSI );
    gpio_reset_pin( PIN_NUM_CLK );
    gpio_reset_pin( PIN_NUM_CS );
    gpio_reset_pin( PIN_NUM_DC );
    gpio_reset_pin( PIN_NUM_RST );
    gpio_reset_pin( PIN_NUM_BCKL );

    lcd_spi_init();

    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_BCKL, 1);

    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);



    write_command(0x01);
    vTaskDelay(100 / portTICK_RATE_MS);

    write_command(0x11);
    vTaskDelay(100 / portTICK_RATE_MS);

    /* RGB 5-6-5-bit  */
    write_command(0x3A);
    write_data(0x55);
    vTaskDelay(100 / portTICK_RATE_MS);

    write_command(0x36);
    write_data(0x00);
 

    write_command(0x2a);
    write_data(0x00);
    write_data(0x00);
    write_data(0x00);
    write_data(0xF0);
 

    write_command(0x2B);
    write_data(0x00);
    write_data(0x00);
    write_data(0x00);
    write_data(0xF0);

    write_command(0x21);
    vTaskDelay(100 / portTICK_RATE_MS);

    write_command(0x13);
    vTaskDelay(100 / portTICK_RATE_MS);    

    write_command(0x29);
    vTaskDelay(100 / portTICK_RATE_MS); 
}
void set_addr_window(uint16_t x_0, uint16_t y_0, uint16_t x_1, uint16_t y_1)
{
    uint8_t data[4];

    if (x_0 > x_1) {
        x_0 = x_0 ^ x_1;
        x_1 = x_0 ^ x_1;
        x_0 = x_0 ^ x_1;
    }
    if (y_0 > y_1) {
        y_0 = y_0 ^ y_1;
        y_1 = y_0 ^ y_1;
        y_0 = y_0 ^ y_1;
    }

    write_command(0x2A);
    data[0] = x_0 >> 8;
    data[1] = x_0;
    data[2] = x_1 >> 8;
    data[3] = x_1;
    lcd_data(spi,data, 4);
    write_command(0x2b);
    data[0] = y_0 >> 8;
    data[1] = y_0;
    data[2] = y_1 >> 8;
    data[3] = y_1;
    lcd_data(spi,data, 4);
    write_command(0x2c);
}
void set_addr_window(uint16_t x_0, uint16_t y_0, uint16_t x_1, uint16_t y_1)
{
    uint8_t data[4];

    if (x_0 > x_1) {
        x_0 = x_0 ^ x_1;
        x_1 = x_0 ^ x_1;
        x_0 = x_0 ^ x_1;
    }
    if (y_0 > y_1) {
        y_0 = y_0 ^ y_1;
        y_1 = y_0 ^ y_1;
        y_0 = y_0 ^ y_1;
    }

    write_command(0x2A);
    data[0] = x_0 >> 8;
    data[1] = x_0;
    data[2] = x_1 >> 8;
    data[3] = x_1;
    lcd_data(spi,data, 4);
    write_command(0x2b);
    data[0] = y_0 >> 8;
    data[1] = y_0;
    data[2] = y_1 >> 8;
    data[3] = y_1;
    lcd_data(spi,data, 4);
    write_command(0x2c);
}
// void st7789_area_draw(uint16_t x,
//                       uint16_t y,
//                       uint16_t width,
//                       uint16_t height,
//                       uint8_t *frame,
//                       uint32_t areaSize)
// {
//     uint16_t *rgb565_frame = (uint16_t *)frame;

//     if (width * height != areaSize) {
//         printf("error parm width * height != areaSize\n");
//         return;
//     }

//     set_addr_window(x, y, x + width - 1, y + height - 1);
//     uint32_t bufferSize = width * height;
//     unsigned char *burst_buffer = (unsigned char *)malloc(bufferSize * 2);
//     for (uint32_t i = 0; i < bufferSize; i++) {
//         burst_buffer[2 * i]     = rgb565_frame[i] >> 8;
//         burst_buffer[2 * i + 1] = rgb565_frame[i];
//     }
//     lcd_data(spi,burst_buffer, bufferSize * 2);
//     free(burst_buffer);
// }
void st7789_rect_draw(uint16_t x,
                      uint16_t y,
                      uint16_t width,
                      uint16_t height,
                      uint16_t color)
{
    uint8_t xxx=55;
    set_addr_window(x, y,x+width,y+height);
    write_command(0x2c);
    for(uint16_t i=0;i<width*height;i++)
    {
        lcd_data(spi,&xxx,1);
        lcd_data(spi,&xxx,1);
    }
}
void st7789_rect_draw_black(void)
{
    uint8_t xxx=0xff;
    set_addr_window(0, 0,240,320);
    write_command(0x2c);
    for(uint32_t i=0;i<240*320*2;i++)
    {
        lcd_data(spi,&xxx,1);
        lcd_data(spi,&xxx,1);
    }
}