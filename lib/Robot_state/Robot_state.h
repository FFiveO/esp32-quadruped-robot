#pragma once

#include <Arduino.h>



enum RobotState{                  //机器人状态定义
  STAND,
  LIE,
  TRANSITION,
  FALLEN
};


extern RobotState robot_state;    //机器人当前状态

extern String RSCN[20];           //“数值状态”  映射  “中文”