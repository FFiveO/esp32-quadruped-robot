#pragma once

#include <Arduino.h>

#include "Types.h"

extern float Back_Leg_runing_IK_offset;

void Motion_update();

void Motion_stop();

void Motion_resetPose();

void Motion_startMarching(float step, float turn);

void Motion_setRealtimeControl(float joyX,
                               float joyY,
                               float speedScale,
                               float bodyHeight,
                               float bodyPitch,
                               float bodyRoll,
                               float liftScale);

bool Motion_runAction(String actionName);//扩展运动函数

bool Motion_isWalking();//是否在运动

bool Motion_Checked_Rob_Stop();


bool Motion_setGait(GaitType gait);

bool Motion_setGaitByName(String gaitName);

GaitType Motion_getGait();

const char* Motion_getGaitName();


static float applyDeadZone(float value, float deadZone);


//--------------------------------------------------------------------------------------------------------------------------------------------------
float get_SpeedScale();
float get_bodyHeightOffset();
float get_bodyPitchOffset();
float get_bodyRollOffset();
float get_liftScale();

