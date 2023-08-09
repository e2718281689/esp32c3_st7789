#ifndef MPU_6050_H
#define MPU_6050_H
#include <stdio.h>

void Mpu6050_init();
void MPU6050ReadAcc(uint16_t *accData);
void MPU6050ReadGyro(uint16_t *gyroData);
void MPU6050ReadTemp(float *tempData);
#endif /*MPU_6050_H*/