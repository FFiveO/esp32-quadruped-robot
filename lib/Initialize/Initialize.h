#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "Types.h"

extern Leg leg[4];
//系统初始化
void System_Initialize();

//初始化舵机参数,结构体并且让机器狗进入趴下的状态
void hardware_begin();



