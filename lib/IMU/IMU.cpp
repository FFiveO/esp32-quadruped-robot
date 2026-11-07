#include "IMU.h"

#include <Wire.h>

#include "RobotConfig.h"
#include "BalanceController.h"

//记录机器人当前的左右倾斜角（Roll）和前后俯仰角（Pitch）。
//本机器人实际情况是 Roll反映前后   Pitch反映左右
static float roll = 0.0f;
static float pitch = 0.0f;
static float gyroXrate = 0.0f;
static float gyroYrate = 0.0f;


static float alpha = 0.96f;//这是互补滤波算法的“信任系数”。0.96 意味着我们更相信陀螺仪的短期变化（占96%），只用加速度计（占4%）来做微小的纠正。

// 当前希望机器人保持的姿态
// 开启平衡模式时自动记录
static float targetRoll = 0.0f;
static float targetPitch = 0.0f;



//--------------------------------------------------------------------------------------------------------------------------------

//求取加速度
static bool IMU_readAccel(int16_t* x, int16_t* y, int16_t* z) {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(ACCEL_XOUT_H);

  if (Wire1.endTransmission(false) != 0) {
    return false;
  }

  if (Wire1.requestFrom(MPU_ADDR, 6, true) != 6) {
    return false;
  }

  *x = (Wire1.read() << 8) | Wire1.read();
  *y = (Wire1.read() << 8) | Wire1.read();
  *z = (Wire1.read() << 8) | Wire1.read();

  return true;
}
//求取角速度
static bool IMU_readGyro(int16_t* x, int16_t* y, int16_t* z) {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(GYRO_XOUT_H);

  if (Wire1.endTransmission(false) != 0) {
    return false;
  }

  if (Wire1.requestFrom(MPU_ADDR, 6, true) != 6) {
    return false;
  }

  *x = (Wire1.read() << 8) | Wire1.read();
  *y = (Wire1.read() << 8) | Wire1.read();
  *z = (Wire1.read() << 8) | Wire1.read();

  return true;
}


//IMU初始化
void IMU_begin() {
  Wire1.beginTransmission(MPU_ADDR);//建立连接
  Wire1.write(PWR_MGMT_1);//写入要前往的地址
  Wire1.write(0x00);
  Wire1.endTransmission(true);//发送数据

  delay(100);

  int16_t ax, ay, az;

  if (IMU_readAccel(&ax, &ay, &az)) {
    roll = atan2((float)ay, (float)az) * 57.296f;
    pitch = atan2(-(float)ax, sqrt((float)ay * ay + (float)az * az)) * 57.296f;

    Serial.println("陀螺仪 MPU6050 初始化完成");
  } else {
    Serial.println("陀螺仪 MPU6050 初始化读取失败");
  }
}


//根据Roll和Pitch的值判断是否跌倒:只要有一方>35°就认为摔倒
bool IMU_isFallen() {
  return abs(roll) > FALLEN_ANGLE || abs(pitch) > FALLEN_ANGLE;
}


//实时获取roll和pitch
void IMU_update() {
  static unsigned long lastMicros = micros();//获取系统微秒时间戳

  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastMicros) / 1000000.0f;  //本次和上次执行间隔的秒数(以微秒为单位)
  lastMicros = nowMicros;

  if (dt <= 0.0f || dt > 0.1f) {//间隔异常的时候强制用0.01s，防止程序卡顿导致角度直接飞掉。
    dt = 0.01f;
  }

  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  if (!IMU_readAccel(&ax, &ay, &az)) {//获取三轴加速度
    return;
  }

  if (!IMU_readGyro(&gx, &gy, &gz)) {//获取三轴陀螺仪
    return;
  }

  //互补滤波:陀螺仪负责短时动态，加速度计负责修正漂移
  
  //利用重力在三轴的分量，算出当前倾斜角度（roll 横滚、pitch 俯仰）
  float accRoll = atan2((float)ay, (float)az) * 57.296f;
  float accPitch = atan2(-(float)ax, sqrt((float)ay * ay + (float)az * az)) * 57.296f;
  //求出的roll 和 pitch: 机器人一动、有震动、加速，数值就会不准，噪声大。但不会随时间漂移，静态平放时角度很准。
  //不能完全相信
 
 
  //陀螺仪原始值转角速度
  gyroXrate = gx / 131.0f; //x轴角速度
  gyroYrate = gy / 131.0f; //y轴角速度

  //最终角度 = α × 陀螺仪积分输出 + (1‑α) × 加速度计算出角度
  roll = alpha * (roll + gyroXrate * dt) + (1.0f - alpha) * accRoll;
  pitch = alpha * (pitch + gyroYrate * dt) + (1.0f - alpha) * accPitch;
}


//返回当前机器人Roll(前后姿态)
float IMU_getRoll() {
  return roll;
}
//返回当前机器人Pitch(左右姿态)
float IMU_getPitch() {
  return pitch;
}

float IMU_getGyroXRate() {
    return gyroXrate;
}

float IMU_getGyroYRate() {
    return gyroYrate;
}

float Show_Imu_Data() {
  static unsigned long imuDebugTimer = 0;

  //实时打印imu数据
  if(millis() - imuDebugTimer > 50)
  {
      imuDebugTimer = millis();

      Serial.print("Roll:");
      Serial.print(IMU_getRoll());
      Serial.print(",");
      Serial.print("Pitch:");
      Serial.println(IMU_getPitch());
      
      Serial.print(",GyroX:");
      Serial.print(IMU_getGyroXRate());
      Serial.print(",GyroY:");
      Serial.println(IMU_getGyroYRate());
  }
}