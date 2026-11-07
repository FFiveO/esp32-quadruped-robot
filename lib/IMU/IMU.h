#pragma once

#include <Arduino.h>

void IMU_begin();//初始化



bool IMU_isFallen();//摔到了吗


void IMU_update();//获取当前的roll 和 pitch

float IMU_getRoll();//get函数 返回 Roll

float IMU_getPitch();//get函数 返回 pitch

float IMU_getGyroXRate();

float IMU_getGyroYRate();


float Show_Imu_Data();//实时输出IMU数据, loop里调用