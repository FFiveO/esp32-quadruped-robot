#include "Initialize.h"

#include "RobotConfig.h"
#include "DisplayUI.h"
#include "ServoDriver.h"
#include "IMU.h"
#include "Motion.h"
#include "WebControl.h"
#include "Command.h"
#include "Robot_state.h"
#include "Test_Leg.h"
#include "ActionController.h"
#include "VoiceManager.h"
#include "BoredSystem.h"

Leg leg[4];//四条腿的结构体代码

Angle_Action_Mode Leg_action_life_cycle[4]; //四条腿的运动周期控制代码

//只负责"系统启动顺序"
void System_Initialize(){

  Serial.begin(115200);//串口波特率(通信速率)：115200 代表串口线上数据传输速度(管数据传得快不快)

  Wire.begin(PCA_SDA, PCA_SCL);//初始化 I2C PCA9685
  Wire1.begin(MPU_SDA, MPU_SCL);//初始化 I2C MPU6050 

  DisplayUI_begin();//初始化 OLED
  
  ServoDriver_begin();//初始化舵机驱动板

  IMU_begin();//初始化 MPU6050

  Voice_begin(); //初始化扬声器模块

  BoredSystem_begin(); //初始化无聊模式

  WebControl_begin(Command_execute);//启动网页服务器

  hardware_begin();//初始化每个腿的参数等等



  robot_state = LIE;  //上电后机状态为"LIE"
  isTestMode = false; //默认不在测试状态

  Serial.println("系统启动完成");
}


void hardware_begin(){//初始化舵机参数,结构体并且让机器狗进入趴下的状态
  
  
  //八个舵机初始化
  ServoInfo servo[8] = {
    
    {0, 100, 540, OFF_RBH},  // 右后大腿
    {1, 100, 540, OFF_RBK},  // 右后小腿

    {2, 100, 540, OFF_LBH},  // 左后大腿
    {3, 100, 535, OFF_LBK},  // 左后小腿

    {4, 100, 540, OFF_LFH},  // 左前大腿
    {5, 100, 535, OFF_LFK},  // 左前小腿

    {6, 100, 525, OFF_RFH},  // 右前大腿
    {7, 100, 540, OFF_RFK},  // 右前小腿
    
  };


  //四条腿的生命周期初始化(以初始状态为0,表示一开始不用这个生命周期)(IK坐标系控制的)
  Ik_Mode Ik_state[4] = {
    {0.0f,  0.0f,  0.0f, 0.0f, 0.0f,  0.0f}, //右后
    {0.0f,  0.0f,  0.0f, 0.0f, 0.0f,  0.0f}, //左后
    {0.0f,  0.0f,  0.0f, 0.0f, 0.0f,  0.0f}, //左前
    {0.0f,  0.0f,  0.0f, 0.0f, 0.0f,  0.0f}  //右前
  };


  //四条腿的生命周期初始化(以初始趴下角度设置的)(角度控制的)
  Angle_Action_Mode Leg_action_life_cycle[4] = {
    {0.0,   Action_getStandHip(0) + Stand_to_Lie_Back_Hip_Offset,                    0.0,     0.0,     Action_getStandKnee(0) + Stand_to_Lie_Back_Knee_Offset,                     0.0,    0.0,   0.0,   false},
    {0.0,   Action_getStandHip(1) + Right_to_Left(Stand_to_Lie_Back_Hip_Offset),     0.0,     0.0,     Action_getStandKnee(1) + Right_to_Left(Stand_to_Lie_Back_Knee_Offset),      0.0,    0.0,   0.0,   false},
    {0.0,   Action_getStandHip(2) + Right_to_Left(Stand_to_Lie_Front_Hip_Offset),    0.0,     0.0,     Action_getStandKnee(2) + Right_to_Left(Stand_to_Lie_Front_Knee_Offset),     0.0,    0.0,   0.0,   false},
    {0.0,   Action_getStandHip(3) + Stand_to_Lie_Front_Hip_Offset,                   0.0,     0.0,     Action_getStandKnee(3) + Stand_to_Lie_Front_Knee_Offset,                    0.0,    0.0,   0.0,   false},
  };



  //四条腿初始化(初始状态为趴着,控制模式为ANGLE_MODE)
  // leg[0] 右后
  leg[0].channel = 0;
  leg[0].isLeft=false;
  leg[0].isFront=false;
  leg[0].Hip = servo[0];
  leg[0].Knee = servo[1];
  leg[0].control_mode = ANGLE_MODE;
  leg[0].ik_state = Ik_state[0];
  leg[0].angle_action = Leg_action_life_cycle[0];

  // leg[1] 左后
  leg[1].channel = 1;
  leg[1].isLeft=true;
  leg[1].isFront=false;
  leg[1].Hip = servo[2];
  leg[1].Knee = servo[3];
  leg[1].control_mode = ANGLE_MODE;
  leg[1].ik_state = Ik_state[1];
  leg[1].angle_action = Leg_action_life_cycle[1];

  // leg[2] 左前
  leg[2].channel = 2;
  leg[2].isLeft=true;
  leg[2].isFront=true;
  leg[2].Hip = servo[4];
  leg[2].Knee = servo[5];
  leg[2].control_mode = ANGLE_MODE;
  leg[2].ik_state = Ik_state[2];
  leg[2].angle_action = Leg_action_life_cycle[2];

  // leg[3] 右前
  leg[3].channel = 3;
  leg[3].isLeft=false;
  leg[3].isFront=true;
  leg[3].Hip = servo[6];
  leg[3].Knee = servo[7];
  leg[3].control_mode = ANGLE_MODE;
  leg[3].ik_state = Ik_state[3];
  leg[3].angle_action = Leg_action_life_cycle[3];

  
  
  for(int i=0 ; i<4 ; i++){

    if (leg[i].isLeft == false && leg[i].isFront == false) leg[i].angle_action.current_knee -= Back_Leg_runing_IK_offset;
    if (leg[i].isLeft == true && leg[i].isFront == false) leg[i].angle_action.current_knee += Back_Leg_runing_IK_offset;

    ServoDriver_writeServo(leg[i].Hip, leg[i].angle_action.current_hip);
    ServoDriver_writeServo(leg[i].Knee, leg[i].angle_action.current_knee);

    Serial.print("Leg(");
    Serial.print(i);
    Serial.print(") Hip=");
    Serial.print(leg[i].angle_action.current_hip);
    Serial.print("    knee=");
    Serial.println(leg[i].angle_action.current_knee);
  }

  Serial.println("运动模块初始化完成");
  Serial.println();
  Serial.print("机器人当前状态为: ");
  Serial.println(RSCN[robot_state]);
}
