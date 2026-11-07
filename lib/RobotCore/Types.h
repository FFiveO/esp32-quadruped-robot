#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>



//舵机控制模式
enum LegControlMode {
  IK_MODE,          //IK模式(步态)
  ANGLE_MODE        //静态动作(角度版)
};


//行走步态类型
enum GaitType
{
    GAIT_TROT,      //trot
    GAIT_WALK,      //walk

    GAIT_COUNT      //步态数量，永远放最后
};

//所有动作
enum ActionType
{

    LIE_TO_STAND,   //站起
    STAND_TO_LIE,   //趴下

    WAVE,           //挥手
    PUSH_UP,        //俯卧撑
    PEE,            //小便

    TWIST,          //扭动，使用IK坐标控制

    ACTION_NONE     //无动作
};

//所有动作的子步骤
enum ActionStep
{
    //站立动作
    ENTIRE_LIE_TO_STAND,

    //趴下动作
    ENTIRE_STAND_TO_LIE,
    
    //握手动作
    WAVE_LEG_START_MOVE, 
    WAVE_HAND_ONE_S,
    WAVE_HAND_ONE_E,
    WAVE_HAND_TWO_S,
    WAVE_HAND_TWO_E,
    WAVE_PRE_STAND,//机器人准备站立起来,
    WAVE_LEG_END_MOVE,

    //俯卧撑动作
    PUSH_UP_LEG_START_MOVE,
    PUSH_UP_ONE_S,
    PUSH_UP_ONE_E,
    PUSH_UP_TWO_S,
    PUSH_UP_TWO_E,
    PUSH_UP_THREE_S,
    PUSH_UP_THREE_E,
    PUSH_UP_FOUR_S,
    PUSH_UP_FOUR_E,
    PUSH_UP_CHANGE,
    PUSH_UP_LEG_END_MOVE,

    //撒尿动作
    PEE_LEG_START_MOVE,
    PEE_RB_LEG_RAISE,
    PEE_RBK_ONE_S,
    PEE_RBK_ONE_E,
    PEE_RBK_TWO_S,
    PEE_RBK_TWO_E,
    PEE_RBK_THREE_S,
    PEE_RBK_THREE_E, 
    PEE_LEG_END_MOVE,
    


    //无子动作
    STEP_NONE

};

//串口指令解析
enum RobotCommand {
  CMD_GAIT_CHANGE,            //切换步态切换
  
  CMD_OLED_CHANGE,            //显示屏显示模式更改开关
  CMD_BORED_MODE_CHANGE,      //无聊模式切换开关
  CMD_BALANCE_CHANGE,         //平衡模式开关
  CMD_BALANCE_NORMAL,         //普通平衡模式
  CMD_BALANCE_STRONG,         //加强平衡模式
  
  CMD_RESET,                  //重置机器人指令(回到站立状态标准,且标准化前端页面控制参数)

  CMD_MARCH,                  //踏步指令
  CMD_STOP_MARCH,             //停止踏步指令



  CMD_VOICE,                  //发出声音指令
  CMD_VOLUME,                 //调节音量指令

  CMD_ACTION,                 //动作命令包含很多不同的指令
  
  CMD_INFO,                   //打印信息指令
  CMD_PRINT_ROBOT_STATE,      //打印机器人当前状态指令
  
  CMD_TEST,                   //测试模式指令
  CMD_EXITTEST,               //停止测试指令

  

  CMD_UNKNOWN,                //识别不到指令
  CMD_ERROR                   //指令错误,一般是执行指令前提的状态不匹配,例如要趴下,但是现在正在执行挥手指令(TRANSITION状态)
};


//------------------------------------------------------------------------------------------------------------------------------------------------------------------
//舵机结构体
struct ServoInfo {
  int channel;                //舵机控制通道
  uint16_t min_pwm;           //舵机实际最小pwm值
  uint16_t max_pwm;           //舵机实际最大pwm值
  float angle_offset;         //舵机装配误差值
};


 //角度控制结构体(控制一条腿)
struct Angle_Action_Mode {

  //大腿舵机角度
  float start_hip;            //开始角度
  float current_hip;          //当前角度
  float target_hip;           //目标角度

  //小腿舵机角度
  float start_knee;
  float current_knee;         // 这个是ANGLE_MODE 当前角度
  float target_knee;


  float phase;                //当前阶段

  float duration;             //动作持续时间(毫秒)
  
  bool running;               //当前是否在运动
};

//IK轨迹插值器的运行状态
struct IkActionState{
  float start_x;              //开始x
  float start_z;              //开始z

  float target_x;             //目标x
  float target_z;             //目标z

  float phase;                //当前阶段
  float duration;             //动作持续时间(毫秒)
  bool running;               //当前是否在运动
};

//IK控制结构体
struct Ik_Mode{
    float x;                  //足端在坐标系的x,z
    float z;

    float hip_angle;          //x对应的大腿关节角度
    float knee_angle;         //z对应的小腿关节角度

    float x_offset;           //x在坐标系的偏移
    float z_offset;           //z在坐标系的偏移
};



//机器狗腿结构体
struct Leg {
  uint16_t channel;                         //腿编号

  bool isLeft;                              //是否为左腿
  bool isFront;                             //是否为前腿吗
  LegControlMode control_mode;              //控制腿当前的运动模式


  ServoInfo Hip;                            //大腿舵机
  ServoInfo Knee;                           //小腿舵机
  
  Ik_Mode ik_state;                         //IK运动模式下的参数
  Angle_Action_Mode angle_action;           //角度直驱下的运动参数
  IkActionState ik_action;                  //IK_MODE下的静态轨迹插值状态
};


//波浪机身晃动，独立时间轴，不依赖ActionStep状态机
typedef struct{
    bool active;               // 是否正在执行波浪动作
    unsigned long tStartMs;    // 动作开始时刻 ms
    float durationTotal;       // 整套动作总时长 s

    float legDelay[4];         // 每条腿延时启动时间(s)
    float legMoveDur[4];       // 单条腿踮脚运动时间(s)

    float zBase;               //基准Z
    float zPeak;               //踮脚最高Z
}BodyWaveAction;

extern BodyWaveAction bodyWave;

#endif