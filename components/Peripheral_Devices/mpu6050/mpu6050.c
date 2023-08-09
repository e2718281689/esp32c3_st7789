#include "esp_log.h"
#include "driver/i2c.h"
#include "mpu6050.h"
static const char * TAG="mpu6050";
#define Mpu6050_SENSOR_ADDR 0x68
#define I2C_MASTER_NUM 0
static esp_err_t Mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, Mpu6050_SENSOR_ADDR, &reg_addr, 1, data, len, 100 / portTICK_RATE_MS);
}
static esp_err_t Mpu6050_register_write(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, Mpu6050_SENSOR_ADDR, write_buf, sizeof(write_buf), 100 / portTICK_RATE_MS);

    return ret;
}
void Mpu6050_init()
{
    Mpu6050_register_write(0x6B, 0x00);	    //解除休眠状态
	Mpu6050_register_write(0x19 , 0x07);	    //陀螺仪采样率，1KHz
	Mpu6050_register_write(0x1A , 0x06);	        //低通滤波器的设置，截止频率是1K，带宽是5K
	Mpu6050_register_write(0x1C , 0x00);	  //配置加速度传感器工作在2G模式，不自检
	Mpu6050_register_write(0x1B, 0x18);     //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
}
/**
  * @brief   读取MPU6050的加速度数据
  * @param   
  * @retval  
  */
void MPU6050ReadAcc(uint16_t *accData)
{
    uint8_t buf[6];
    Mpu6050_register_read(0x3B, buf, 6);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}

/**
  * @brief   读取MPU6050的陀螺仪数据
  * @param   
  * @retval  
  */
void MPU6050ReadGyro(uint16_t *gyroData)
{
    uint8_t buf[6];
    Mpu6050_register_read(0x43,buf,6);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}


/**
  * @brief   读取MPU6050的原始温度数据
  * @param   
  * @retval  
  */
void MPU6050ReadTemp(float *tempData)
{
	uint8_t buf[2];
  short raw;
  Mpu6050_register_read(0x41,buf,2);     //读取温度值
  raw = (buf[0] << 8) | buf[1];
  *tempData=36.53+((double)raw)/340; 

}
