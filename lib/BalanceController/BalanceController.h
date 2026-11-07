#pragma once

#include <Arduino.h>


//平衡档位
enum BalanceModeLevel {
    BALANCE_NORMAL,   //普通平衡，现有参数
    BALANCE_STRONG    //超强平衡，大增益，适合人为晃动演示
};

struct BalanceOffset
{
    float x[4];
    float z[4];
};

void BalanceController_setEnabled();

bool BalanceMode_get();

void BalanceController_setLevel(BalanceModeLevel lv);

BalanceOffset BalanceController_update(bool stance[4]);
