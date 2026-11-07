#pragma once

#include <Arduino.h>

#include "Types.h"

//根据腿里面的x,z,为该腿算出对应Hip舵机和Knee舵机的角度
bool Leg_IK(Leg &leg);

//根据情况修正传入的x,z,并设置为Leg数组的值,再运动学转换为舵机角度,然后设置转动
void Leg_IK_move(Leg &leg, float x, float z);//包工头:根据工人的结果来交差(驱动舵机)
