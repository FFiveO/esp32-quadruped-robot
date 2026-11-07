#pragma once

#include <Arduino.h>

//WiFi参数==========================================================================================================
static const char WIFI_SSID[] = "YueDong_Robot";
static const char WIFI_PASSWORD[] = "123456789";

//I2C引脚===========================================================================================================
#define PCA_SDA 19   
#define PCA_SCL 23

#define MPU_SDA 33
#define MPU_SCL 32

//OLED参数===========================================================================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

//MPU6050寄存器=======================================================================================================
#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

//机器狗几何参数=======================================================================================================
static const float L1 = 80.0f;
static const float L2 = 71.0f;

static const float STAND_X = 0.0f;
static const float STAND_Z = -80.0f;

static const float IK_HIP_OFFSET = 37.24f;  //运动学模型非常重要的一部分:(把数学坐标系转换为机器人实际坐标系)
static const float IK_KNEE_OFFSET = 60.0f;
//机器人规定动作角度====================================================================================================

//注意机器狗的趴下角度是由标准站立角度 + 偏移量(自测构成的)
//未来扩展动作的角度也是由此构成
/*static const float Standard_Leg_Stand_Angle[2][4]={//默认站立角度(数组顺序右后(0)  -->  左后(1)  -->  左前(2)  -->  右前(3))
  {74.56,    105.44,   105.44,   74.56},  //大腿
  {123.66,   56.34,   56.34,   123.66}  //小腿
};*/

static const float Standard_Leg_Stand_Angle[2][4]={//默认站立角度(数组顺序右后(0)  -->  左后(1)  -->  左前(2)  -->  右前(3))
  {70.34,    109.66,   105.44,   74.56},  //大腿
  {123.86,   56.14,   56.34,   123.66}  //小腿
};

//躺下舵机变化
static const float Stand_to_Lie_Front_Hip_Offset = -30.0f;     //躺下: 前大腿膝关节舵机变化
static const float Stand_to_Lie_Front_Knee_Offset = -75.0f;   //躺下: 前小腿踝关节舵机变化
static const float Stand_to_Lie_Back_Hip_Offset = -30.0f;      //躺下: 后大腿膝关节舵机变化
static const float Stand_to_Lie_Back_Knee_Offset = -80.0f;    //躺下: 后小腿踝关节舵机变化

//站立舵机变化:当前角度(以躺下角度(标准站立+躺下舵机偏移) + 站立偏移)  本质就是还原到Stand_Leg_Stand_Angle数组的角度
static const float Lie_to_Stand_Front_Hip_Offset = 30.0f;     //站起: 前大腿膝关节舵机变化
static const float Lie_to_Stand_Front_Knee_Offset = 75.0f;   //站起: 前小腿踝关节舵机变化
static const float Lie_to_Stand_Back_Hip_Offset = 30.0f;      //站起: 后大腿膝关节舵机变化
static const float Lie_to_Stand_Back_Knee_Offset = 80.0f;    //站起: 后小腿踝关节舵机变化



//舵机偏移==============================================================================================================

//左大腿: 前进-  后退+
//左小腿: 上抬-  下放+
//右大腿: 前进+  后退-
//右小腿: 上抬+  下放-


#define OFF_RBH          4.0f  //右后大
#define OFF_RBK          4.0f
#define OFF_LBH          -9.0f   //左后大
#define OFF_LBK          0.0f 
#define OFF_LFH          3.0f  //左前大
#define OFF_LFK          3.0f
#define OFF_RFH          0.0f   //右前大
#define OFF_RFK          2.0f


//舵机PWM参数===========================================================================================================

static const int SERVO_FREQ = 50;

//步态参数=====================================================info=========================================================


static const float DEFAULT_STEP = 25.0f;//步长
static const float DEFAULT_CYCLE_TIME = 0.5f;//一次抬起放下的时间
static const float DEFAULT_LIFT_HEIGHT = 35.0f;//抬腿高度

static const float TURN_LEFT = -40.0f;
static const float TURN_RIGHT = 40.0f;
static const float TURN_SPEED = 35.0f;

//平衡控制参数============================================================================================================


// ============================================================================
// IMU 姿态平衡参数
//
// 单位解释：
//
// KP:
//   mm / degree
//   身体偏多少度，产生多少毫米足端 Z 修正
//
// KD:
//   mm / (degree/s)
//   身体正在以多快的角速度倾倒，用它产生“阻尼”
//
// MAX_BALANCE_OFFSET:
//   最终单轴最大补偿量，单位 mm
// ============================================================================

// 小于这个角度认为属于正常步态晃动
static const float BALANCE_DEAD_ZONE = 1.5f;

// 非常重要：不要让 IMU 大幅改写足端轨迹
static const float MAX_BALANCE_OFFSET = 3.0f;

static const float FALLEN_ANGLE = 35.0f;

//定时参数===============================================================================================================

static const unsigned long OLED_INTERVAL_MS = 200; //oled刷新时间
static const unsigned long DEBUG_INTERVAL_MS = 300;
static const int MAIN_LOOP_DELAY_MS = 5;
