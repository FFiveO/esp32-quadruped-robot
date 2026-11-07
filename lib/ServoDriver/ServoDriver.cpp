#include "ServoDriver.h"

#include "Test_Leg.h"
#include "Initialize.h"
#include <Adafruit_PWMServoDriver.h>
#include "RobotConfig.h"
#include "Robot_state.h"

static Adafruit_PWMServoDriver pwm;

static float lastAngle[16]={0};

//舵机驱动板初始化
void ServoDriver_begin() {
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  delay(10);

  Serial.println("舵机驱动板 PCA9685 初始化完成");
}

//计算当前舵机在angle时对应的pwm值(包含舵机机械误差)
int ServoDriver_angleToPWM(const ServoInfo &servo, float angle) {
  angle += servo.angle_offset;              //角度要根据舵机的误差进行调整
  angle = constrain(angle, 5.0f, 175.0f);   //角度进行限制

  float pwmValue = servo.min_pwm +(servo.max_pwm - servo.min_pwm) * angle / 180.0f;

  return (int)pwmValue;
}

//驱动当前舵机前往angle角度
void ServoDriver_writeServo(const ServoInfo &servo, float angle) {

  if(abs(lastAngle[servo.channel]-angle)<0.5)return;

  lastAngle[servo.channel]=angle;

  int pwmValue = ServoDriver_angleToPWM(servo, angle); //获取当前舵机的在angle下的pwm值是多少
  pwm.setPWM(servo.channel, 0, pwmValue);
}

