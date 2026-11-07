#include "LegKinematics.h"

#include "RobotConfig.h"
#include "ServoDriver.h"

//根据腿里面的x,z,为该腿算出对应Hip舵机和Knee舵机的角度(x,z为目标点坐标)
bool Leg_IK(Leg &leg) {
  float x1 = -leg.ik_state.x;
  float y1 = -leg.ik_state.z;

  float d2 = x1 * x1 + y1 * y1;
  float d = sqrt(d2);

  if (d < 0.1f) {
    Serial.println("IK失败: 目标点距离过小");
    return false;
  }

  float minReach = abs(L1 - L2) + 1.0f;
  float maxReach = L1 + L2 - 1.0f;

  if (d < minReach || d > maxReach) {
    Serial.print("IK失败: 目标点超出范围，腿号=");
    Serial.print(leg.channel);
    Serial.print(" X=");
    Serial.print(leg.ik_state.x);
    Serial.print(" Z=");
    Serial.println(leg.ik_state.z);
    return false;
  }

  float cosKnee = (d2 - L1 * L1 - L2 * L2) / (-2.0f * L1 * L2);
  cosKnee = constrain(cosKnee, -1.0f, 1.0f);

  float shank1 = PI - acos(cosKnee);

  float cosFail = (L1 * L1 + d2 - L2 * L2) / (2.0f * L1 * d);
  cosFail = constrain(cosFail, -1.0f, 1.0f);

  float fail = acos(cosFail);
  float ham1;

  if (x1 > 0) {
    ham1 = abs(atan(y1 / x1)) - fail;
  } else if (x1 < 0) {
    ham1 = PI - abs(atan(y1 / x1)) - fail;
  } else {
    ham1 = PI - 1.5707f - fail;
  }

  float raw_hip = degrees(ham1);
  float raw_knee = 180.0f - degrees(shank1);

  //算出来的关节要做一个真实机器人角度的对照, 比如算出来是56,但是想要的是 90,就增加补偿34
  float calibrated_hip = raw_hip + IK_HIP_OFFSET;
  float calibrated_knee = raw_knee + IK_KNEE_OFFSET;

  //如果是左边腿就要进行一个左右镜像翻转
  if (leg.isLeft) {
    leg.ik_state.hip_angle = 180.0f - calibrated_hip;
    leg.ik_state.knee_angle = 180.0f - calibrated_knee;
  } else {
    leg.ik_state.hip_angle = calibrated_hip;
    leg.ik_state.knee_angle = calibrated_knee;
  }

  return true;
}

//调用代码的时候,Leg结构体里面的x,z还是当前位置的x,z, 而不是targetx和targetz
//因为targetx和targetz是要根据硬件的差异和当前平衡情况适时调整的
void Leg_IK_move(Leg &leg, float x, float z) {
  float final_x = x + leg.ik_state.x_offset;
  float final_z = z + leg.ik_state.z_offset;

  //最终的坐标---->这时候该腿对应结构体里面的x,z才是targetx和targetz
  leg.ik_state.x = final_x;
  leg.ik_state.z = final_z;


  if (Leg_IK(leg)) {//---->这里通过IK将结构体内部的x,z求解出对应的关节角度
    ServoDriver_writeServo(leg.Hip, leg.ik_state.hip_angle);  //驱动大腿舵机
    ServoDriver_writeServo(leg.Knee, leg.ik_state.knee_angle); //驱动小腿舵机
  }
}
