#pragma once

#include <Arduino.h>



void BoredSystem_begin();           //无聊系统初始化

void BoredSystem_update();          //无聊系统更新

void BoredSystem_notifyActivity();  //机器人做过动作, 重置无聊计时

void BoredSystem_setMode();

bool BoredSystem_getMode();


void BoredSystem_notifyExpressionFinished();

bool BoredSystem_isBoredEvent();    //获取是否触发无聊事件标记

void BoredSystem_clearBoredEvent(); //消费事件（外部处理完事件后，清空标记）

void BoredSystem_EventAction();

