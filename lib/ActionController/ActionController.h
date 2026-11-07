#pragma once

#include <Arduino.h>
#include "Types.h"
#include "Robot_state.h"

void Action_control(String action_type);//所有动作的开始

void Action_update();//更新所有动作

void Update_BodyWave();

void Start_Action_Step();//设置每一个动作子步骤的运动角度区间

void Action_NextStep();//一个动作步骤结束后进入的函数(主要是为了多步骤的动作)

bool Leg_isAllFinished();//检查所有腿是不是都停下来了

float Right_to_Left(float angle);





//设置目标为站起stand_to_lie
void set_TO_STAND_START();
//设置目标为趴下lie_to_stand
void set_TO_LIE_START();

void Action_End(RobotState flag);

//挥手一系列动作
void set_WAVE_ALL_LEG_START();
void set_WAVE_R_HANG_ONE_S();
void set_WAVE_R_HANG_ONE_E();
void set_WAVE_R_HANG_TWO_S();
void set_WAVE_R_HANG_TWO_E();
void set_WAVE_ALL_LEG_END();



//俯卧撑一系列动作
void set_PUSH_UP_LEG_START_MOVE();
void set_PUSH_UP_RAISE();
void set_PUSH_UP_LOWER();
void set_PUSH_UP_CHANGE();
void set_PUSH_UP_LEG_END_MOVE();



//撒尿动作
void set_PEE_LEG_START_MOVE();
void set_PEE_RB_LEG_RAISE();
void set_PEE_RBK_BACK();
void set_PEE_RBK_FORWARD();

void set_PEE_LEG_END_MOVE();



//设置一条腿转动目标和时间的工具
void set_ONE_LEG_ANGLE(int LegId, float H_S, float H_E, float K_S, float K_E, float duration);


//获取第i条腿标准大腿关节站立角度
float Action_getStandHip(int legID);

//获取第i条腿标准小腿关节站立角度
float Action_getStandKnee(int legID);


void Start_BodyWave(void);




