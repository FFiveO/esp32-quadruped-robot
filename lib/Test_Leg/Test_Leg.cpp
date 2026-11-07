#include "Test_Leg.h"

#include "LegKinematics.h"
#include "RobotConfig.h"
#include "Initialize.h"
#include "ServoDriver.h"
#include "PrintDebug.h"
#include "ActionController.h"

bool isTestMode = false;              //是否为测试模式

//外部接口给Command调用的,用于处理指令,下达测试命令
void Test_Control(String cmd){

    if(cmd.startsWith("ik") || cmd.startsWith("IK")){                           //IK模式

        int index1 = cmd.indexOf(',');
        int index2 = cmd.indexOf(',', index1+1);
        int index3 = cmd.indexOf(',', index2+1);

        if(index1>0 && index2>0 && index3>0)
        {
            int legID = cmd.substring(index1+1,index2).toInt();
            float x = cmd.substring(index2+1,index3).toFloat();
            float z = cmd.substring(index3+1).toFloat();
            isTestMode=true;
            Test_IK(legID,x,z);
        }else{
            Serial.println("格式错误:");
            Serial.println("IK(ik),腿号,x,z");    
        }

    }else if(cmd.startsWith("angle") || cmd.startsWith("ANGLE")){                //ANGLE模式

        int index1=cmd.indexOf(',');
        int index2=cmd.indexOf(',',index1+1);
        int index3=cmd.indexOf(',',index1+1);

        if(index1>0 && index2>0 && index3>0)
        {
            int legID=cmd.substring(index1+1,index2).toInt();       //具体某条腿
            int jointID=cmd.substring(index2+1,index3).toInt();     //大臂 还是 小臂 
            float delta=cmd.substring(index3+1).toFloat();
            isTestMode=true;
            Test_ServoAngle(legID,jointID,delta);
        }else{
            Serial.println("格式错误:");
            Serial.println("ANGLE(angle),腿号,x,z");    
        }

    }else if(cmd.startsWith("ik_all_leg") || cmd.startsWith("IK_ALL_LEG")){       //IK_ALL_LEG模式

        int p1 = cmd.indexOf(',');

        if(p1 > 0)
        {
            float x = cmd.substring(p1 + 1, cmd.indexOf(',', p1 + 1)).toFloat();
            float z = cmd.substring(cmd.indexOf(',', p1 + 1) + 1).toFloat();
            isTestMode=true;
            Test_IK_ALL_LEG(x,z);
        }else{
            Serial.println("格式错误:");
            Serial.println("IK_ALL_LEG(ik_all_leg),x,z");
        }

    }
}


//退出测试模式
void Exit_Text_Mode(){
    isTestMode = false;
    Serial1.println("测试模式已退出");
}


//IK测试模式
void Test_IK(int legID, float x, float z){
  if (legID < 0 || legID >= 4)
  {
    Serial.println("IK错误: 腿编号范围0-3");
    return;
  }

  Serial.print("测试IK 腿:");
  Serial.print(legID);

  Serial.print(" X=");
  Serial.print(x);

  Serial.print(" Z=");
  Serial.println(z);

  Leg_IK_move(leg[legID], x, z);

  Leg_print(leg[legID]);
}

//ANGLE测试模式
void Test_ServoAngle(int legID,int jointID,float delta){

    if(legID<0 || legID>=4)
    {
        Serial.println("错误:腿编号0-3");
        return;
    }
    
    Leg &targetLeg = leg[legID];


    float Delta = (leg[legID].isLeft) ? Right_to_Left(delta) : delta;   //左边腿做一个角度镜像
    
    if(jointID==0)// 大腿
    {
        
        targetLeg.angle_action.current_hip += Delta;
        ServoDriver_writeServo(targetLeg.Hip, targetLeg.angle_action.current_hip);

        //提示信息
        Serial.print("( ");
        Serial.print(legID);
        Serial.print(" )号腿的大腿角度 在标准角度(站立):");
        Serial.print(Standard_Leg_Stand_Angle[0][legID]);
        Serial.print("的基础上变化了");
        Serial.print(abs(targetLeg.angle_action.current_hip - Standard_Leg_Stand_Angle[0][legID]));
        Serial.print("°(实际输入变量(无镜像,对应机械角度)为");
        Serial.print(delta);
        Serial.print("°),现在的角度是");
        Serial.println(targetLeg.angle_action.current_hip);

    }else if(jointID==1){// 小腿
        
        targetLeg.angle_action.current_knee += Delta;
        ServoDriver_writeServo(targetLeg.Knee, targetLeg.angle_action.current_knee);


        //提示信息
        Serial.print("( ");
        Serial.print(legID);
        Serial.print(" )号腿的小腿角度 在标准角度(站立):");
        Serial.print(Standard_Leg_Stand_Angle[1][legID]);
        Serial.print("的基础上变化了");
        Serial.print(abs(targetLeg.angle_action.current_knee - Standard_Leg_Stand_Angle[1][legID]));
        Serial.print("°(实际输入变量(无镜像,对应机械角度)为");
        Serial.print(delta);
        Serial.print("°),现在的角度是");
        Serial.println(targetLeg.angle_action.current_knee);

    }
    else
    {
        Serial.println("错误:关节0大腿 1小腿");
        return;
    }


}

//IK_ALL_LEG测试模式
void Test_IK_ALL_LEG(float x,float z){

    Serial.print("IK测试站立 X=");
    Serial.print(x);
    Serial.print(" Z=");
    Serial.println(z);


    for(int i=0;i<4;i++)
    {
        leg[i].control_mode = IK_MODE;

        //清除之前offset，避免干扰测试
        leg[i].ik_state.x_offset = 0;
        leg[i].ik_state.z_offset = 0;


        Leg_IK_move(leg[i],x,z);


        Serial.print("leg=");
        Serial.print(i);

        Serial.print("    hip=");
        Serial.print(leg[i].ik_state.hip_angle);

        Serial.print("    knee=");
        Serial.println(leg[i].ik_state.knee_angle);
    }
}
