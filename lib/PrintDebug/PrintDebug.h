#pragma once

#include <Arduino.h>
#include "Types.h"


//打印当前腿的信息
void Leg_print(const Leg &leg);

//打印机器人当前腿部所有的参数
void Print_Leg_Info();

//输出前端控制页面参数
void Print_Front_Controller();

//输出机器人当前状态
void Print_Rs();