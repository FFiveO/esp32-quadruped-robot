#include "HardwareSerial.h"
#include "Types.h"
#include "Command.h"

#include "RobotConfig.h"
#include "LegKinematics.h"
#include "Motion.h"
#include "IMU.h"
#include "Test_Leg.h"
#include "Robot_state.h"
#include "ActionController.h"
#include "BalanceController.h"
#include "PrintDebug.h"
#include "VoiceManager.h"
#include "BoredSystem.h"
#include "DisplayUI.h"


//移动端控制器控制(串口监视器)----->机器人底层标准控制事件
static RobotCommand Command_parse(String cmd) {
  cmd.trim();

  if (cmd.startsWith("gait ") || cmd.startsWith("GAIT ")){        //切换步态
    return CMD_GAIT_CHANGE;
  }



  if( cmd == "display" || cmd == "DISPLAY"){
    return CMD_OLED_CHANGE;                                       //切换显示屏显示模式(显示数据 / 显示表情)
  }

  if( cmd == "bored_mode_change" || cmd == "BORED_MODE_CHANGE"){  //开关无聊模式
    return CMD_BORED_MODE_CHANGE;
  }



  if (cmd == "balance" || cmd == "BALANCE") {                     //开启平衡模式
    if(robot_state == STAND){
      return CMD_BALANCE_CHANGE;
    }else{
      return CMD_ERROR;
    }
  }

   if (cmd == "balance_normal" || cmd == "BALANCE_NORMAL") {       //平衡模式(轻微)
    if(robot_state == STAND){
      return CMD_BALANCE_NORMAL;
    }else{
      return CMD_ERROR;
    }
  } 

  if (cmd == "balance_strong" || cmd == "BALANCE_STRONG") {       //平衡模式(强烈)
    if(robot_state == STAND){
      return CMD_BALANCE_STRONG;
    }else{
      return CMD_ERROR;
    }
  } 


  if (cmd == "reset" || cmd =="RESET") {                          //重置机器人网页控制参数并回到标准站立状态
    if(robot_state == STAND){
      return CMD_RESET;
    }else{
      return CMD_ERROR;
    }
  }





  if (cmd == "march" || cmd == "MARCH") {
    if(robot_state == STAND){                                     //开始踏步
      return CMD_MARCH;
    }else{
      return CMD_ERROR;
    }
  }

  if (cmd == "stop_march" || cmd == "STOP_MARCH") {               //停止踏步
    if(robot_state == STAND ){
      return CMD_STOP_MARCH;
    }else{
      return CMD_ERROR;
    }
  }

  

  if(cmd.startsWith("V ") || cmd.startsWith("v "))                 //发出声音
  {
      return CMD_VOICE;
  }


  if(cmd.startsWith("VOLUME") || cmd.startsWith("volume"))         //音量调整
  {
      return CMD_VOLUME;
  }


//-----------------------------------------------------------------//静态动作

  if(cmd == "lie" || cmd == "LIE"){                                //趴下指令
    if(robot_state == STAND){
      return CMD_ACTION;
    }else{
      return CMD_ERROR;
    }
  }

  if(cmd == "stand" || cmd == "STAND"){                             //站立指令
    if(robot_state == LIE){
      return CMD_ACTION;
    }else{
      return CMD_ERROR;
    }
  }





  if(cmd == "wave" || cmd == "pushup" || cmd == "pee" || cmd == "wavebody"){    //其他动作
     if(robot_state == STAND){
        return CMD_ACTION;
     }else{
        return CMD_ERROR;
     }
  }




 
  if (cmd == "info" || cmd == "INFO") {             //机器人硬件参数打印 
    return CMD_INFO;
  }


  if(cmd == "print_rs"){
    return CMD_PRINT_ROBOT_STATE;                   //打印当前机器人状态
  }




  if(cmd.startsWith("ik") || cmd.startsWith("IK") || 
     cmd.startsWith("angle") || cmd.startsWith("ANGLE") || 
     cmd.startsWith("ik_all_leg") || cmd.startsWith("IK_ALL_LEG")){                //测试模式(IK)
    return CMD_TEST;
  }


  if(cmd=="EXITTEST" || cmd == "exittest"){                        //退出测试模式{
    return CMD_EXITTEST;
  }




  return CMD_UNKNOWN;                                              //输入未知指令
}




//针对"机器人底层标准控制事件"转发任务消息
void Command_execute(String cmd) {

  BoredSystem_notifyActivity();                         //有指令发送,更新无聊模式
  
  cmd.trim();


  RobotCommand command = Command_parse(cmd);            //网页控制(串口控制指令)----->机器人底层标准控制事件

  switch (command) {
  
    case CMD_GAIT_CHANGE:                               //切换步态                
    {
        String gaitName = cmd.substring(5);
        gaitName.trim();

        if(Motion_setGaitByName(gaitName))
        {
            Serial.print(">> 步态切换成功: ");
            Serial.println(Motion_getGaitName());
        }else{
            Serial.println(">> 步态切换失败");
        }

        break;
    }


    
    case CMD_OLED_CHANGE:                               //切换显示屏显示模式(显示数据 / 显示表情)
        DisplayUI_setMode();
        break;

    case CMD_BORED_MODE_CHANGE:                         //无聊模式开关
        BoredSystem_setMode();
        break;



    case CMD_BALANCE_CHANGE:                            //平衡模式开关
        BalanceController_setEnabled();
        break;


    case CMD_BALANCE_NORMAL:
        BalanceController_setLevel(BALANCE_NORMAL);
        // 如果平衡没开，自动打开平衡
        if(!BalanceMode_get()){
            BalanceController_setEnabled();
        }
        break;
    case CMD_BALANCE_STRONG:
        BalanceController_setLevel(BALANCE_STRONG);
        if(!BalanceMode_get()){
            BalanceController_setEnabled();
        }
        break;


    case CMD_RESET:                                     //重置机器人网页控制参数并回到标准站立状态
        Motion_resetPose();
        break;





    case CMD_MARCH:                                     //开始踏步
        if (Motion_runAction(cmd)) {
            Serial.println(">> 原地踏步已启动");
        } else {
            Serial.println(">> 原地踏步启动失败");
        }
        break;

    case CMD_STOP_MARCH:                                //停止踏步
        Motion_stop();
        break;


    
    case CMD_VOICE:                                     //发出声音  
    {
        String voice = cmd.substring(2);
        voice.trim();

        Voice_Button_Control(voice);
        break;
    }
    
  
    case CMD_VOLUME:                                    //音量调整
    {

      String value = cmd.substring(7);

      //音量里 0是最低 1是最高  ,假设为要求改变到30的音量,实际其实是0.3
      float volume =  value.toFloat()/100.0;

      Voice_setVolume(volume);

      break;

    }   





    case CMD_ACTION:                                    //静态动作
        if(robot_state == STAND && Motion_isWalking())
        {
            Motion_stop();
        }
        Action_control(cmd);
        break;

    

    


    case CMD_INFO:                                         //机器人硬件参数打印
        Print_Leg_Info();
        Print_Front_Controller();
        Print_Rs();
        break;


    case CMD_PRINT_ROBOT_STATE:                            //打印当前机器人状态
        if(isTestMode == true){
          Serial.println("当前正在测试模式中,可以输入exittest退出模式");
        }
        Print_Rs();
        break;
        



    
    case CMD_TEST:                                          //测试模式(转发TEST模块处理)
        Test_Control(cmd);
        break;
        
    case CMD_EXITTEST:                                      //退出测试模式
        Exit_Text_Mode();
        break;



    
    case CMD_ERROR:                                         //指令下达失败
        Serial.print("指令下达失败,当前机器人状态为:");
        Serial.println(RSCN[robot_state]);
        break;


    default:                                                //输入未知指令
        Serial.print("未知指令：");                          
        Serial.println(cmd);
        break;
  }
}
          
//监视串口发送数据函数
void Command_processSerial() {
  if (Serial.available() > 0) {//判断串口有没有收到电脑发来的数据，有数据才往下执行
    String input = Serial.readStringUntil('\n');//串口监视器敲完文字，按回车发送，才会把这一行完整读进来
    input.trim();

    if (input.length() > 0) {//如果去掉空格之后不是空字符串, 才执行, 避免只敲回车空行误触发
      Command_execute(input);
    }
  }
}

