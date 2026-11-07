#include "BalanceController.h"
#include "BoredSystem.h"
#include "DisplayUI.h"
#include "HardwareSerial.h"
#include "IMU.h"
#include "Initialize.h"
#include "Motion.h"
#include "PrintDebug.h"
#include "Robot_state.h"
#include "VoiceManager.h"


//打印当前机器人当前腿部所有的参数
void Print_Leg_Info() {
  Serial.println("===========================================================================================================================");
  Serial.println("腿编号| 运动控制模式 ||(IK)|  x  |  z  |  hip_angle | knee_angle  ||(ANGLE)| start_hip | current_hip | target_hip | start_knee | current_knee | target_knee | phase | duration | running");

  for (int i = 0; i < 4; i++) {
    Leg_print(leg[i]);
  }

}

//打印当前腿的信息
void Leg_print(const Leg &leg) {
  Serial.print("   ");
  Serial.print(leg.channel);
  Serial.print("          ");
  Serial.print(leg.control_mode);
  Serial.print("             ");
  Serial.print(leg.ik_state.x);
  Serial.print("      ");
  Serial.print(leg.ik_state.z);
  Serial.print("        ");
  Serial.print(leg.ik_state.hip_angle);
  Serial.print("        ");
  Serial.print(leg.ik_state.knee_angle);
  Serial.print("        ");
  Serial.print(leg.angle_action.start_hip);
  Serial.print("        ");
  Serial.print(leg.angle_action.current_hip);
  Serial.print("        ");
  Serial.print(leg.angle_action.target_hip);
  Serial.print("        ");
  Serial.print(leg.angle_action.start_knee);
  Serial.print("        ");
  Serial.print(leg.angle_action.current_knee);
  Serial.print("        ");
  Serial.print(leg.angle_action.target_knee);
  Serial.print("        ");
  Serial.print(leg.angle_action.phase);
  Serial.print("        ");
  Serial.print(leg.angle_action.duration);
  Serial.print("        ");
  Serial.println(leg.angle_action.running);
}

//输出前端控制页面信息
void Print_Front_Controller(){

  Serial.println("------------------------------前端控制页面--------------------------------");

  Serial.println("<滑杆参数>----------");
  Serial.println("速度 | 高度 | 左右倾 | 前后倾 | 抬腿 | 音量 | 平衡模式 | 显示模式 | 无聊模式");
  Serial.print(get_SpeedScale(), 2);
  Serial.print(" | ");
  Serial.print(get_bodyHeightOffset(), 1);
  Serial.print(" | ");
  Serial.print(get_bodyPitchOffset(), 1);
  Serial.print(" | ");
  Serial.print(get_bodyRollOffset(), 1);
  Serial.print(" | ");
  Serial.print(get_liftScale(), 2);
  Serial.print(" | ");
  Serial.print(Voice_getVolume(), 2);
  Serial.print(" | ");
  Serial.print(BalanceMode_get() ? "ON" : "OFF");
  Serial.print(" | ");
  Serial.print(DisplayUI_getMode() == DISPLAY_NORMAL ? "DATA" : "FACE");
  Serial.print(" | ");
  Serial.println(BoredSystem_getMode() ? "ON" : "OFF");

  Serial.println("--------------------------------------------------------------------------");

}


//输出机器人当前状态
void Print_Rs(){
    Serial.print("机器人当前状态是:");
    Serial.println(RSCN[robot_state]);
}