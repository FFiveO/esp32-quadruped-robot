#include <Arduino.h>

#include "Initialize.h"
#include "Types.h"
#include "Robot_state.h"

#include "IMU.h"
#include "WebControl.h"
#include "Command.h"
#include "Motion.h"
#include "DisplayUI.h"
#include "RobotConfig.h"
#include "ActionController.h"
#include "Test_Leg.h"
#include "VoiceManager.h"
#include "BoredSystem.h"




void setup() {
    System_Initialize();  //系统初始化
    BoredSystem_notifyActivity();

    Serial.println("系统启动完成");
}




void loop() {  
    IMU_update();//读取roll和pitch
    //Show_Imu_Data();//实时输出IMU数据
    
    WebControl_handleClient();//处理网页请求
    Command_processSerial();//处理串口命令

    BoredSystem_update();

    if(BoredSystem_isBoredEvent()){//有无聊事件被触发
        BoredSystem_EventAction();//随机选择表情,并且播放表情
    }



    if(IMU_isFallen()) {//检测机器人的姿态为是否跌倒
        robot_state = FALLEN;
    }

    if(isTestMode == false){//不在测试模式中
        switch(robot_state){
            case STAND:
                Motion_update();
                break;

            case TRANSITION:
                Action_update();
                break;

            case LIE:
                //保持状态,什么也不做
                break;
                
            case FALLEN:
                delay(5000);
                break;
        }
    }
  
    OLED_update();//更新屏幕
    
    delay(MAIN_LOOP_DELAY_MS);
}