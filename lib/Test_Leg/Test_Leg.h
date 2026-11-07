#pragma once

#include <Arduino.h>
#include "Types.h"

extern bool isTestMode;


void Test_Control(String cmd);

void Exit_Text_Mode();

//测试:IK运动学
void Test_IK(int legID, float x, float z);

//测试:舵机角度
void Test_ServoAngle(int legID,int jointID,float delta);

//测试:所有腿的站立x,z
void Test_IK_ALL_LEG(float x,float z);