#pragma once

#include <Arduino.h>
#include "Types.h"

void ServoDriver_begin();//初始化PCA9685

int ServoDriver_angleToPWM(const ServoInfo &servo, float angle);

void ServoDriver_writeServo(const ServoInfo &servo, float angle); //直接根据角度转到舵机

