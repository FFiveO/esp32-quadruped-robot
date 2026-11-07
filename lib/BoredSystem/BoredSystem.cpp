#include "BoredSystem.h"

#include "Robot_state.h"
#include "VoiceManager.h"
#include "Motion.h"
#include "DisplayUI.h"

String Bored_Voice_Name[10] = {"无助", "眼睛闪闪", "开心猫咪叫", "寻找"};



static bool boredEnable=true;               //无聊开关
static int boredLevel=0;                    //无聊程度:0-100

static unsigned long idleStart=0;           //机器人多久没有产生真正的交互/自主事件n 
static unsigned long nextBoredTime=0;       //下一次检查时间
static unsigned long lastBoredIncrease=0;   //无聊值增长计时 

static bool m_boredEventFlag = false;       //无聊事件标记，true=计时条件达成，需要外部处理表情+声音
static bool waitingForExpressionFinish = false;     //等待表情结束状态




void BoredSystem_begin()
{

    randomSeed(millis());
    nextBoredTime = random(10000,15000);
    lastBoredIncrease = millis();
    m_boredEventFlag = false;

}

void BoredSystem_notifyExpressionFinished()
{
    //本次无聊表情已经播放完成
    waitingForExpressionFinish = false;

    //从表情播放结束这一刻开始重新计时
    idleStart = millis();

    //无聊增长计时也从这里继续
    lastBoredIncrease = millis();

    Serial.println("【无聊系统】表情播放结束，重新开始无聊计时");
}

void BoredSystem_update()
{
    //没开启无聊模式,强制退出
    if(!boredEnable)return;
    
    //已经触发了无聊表情,等待表情结束期间,暂停新的无聊计时
    if(waitingForExpressionFinish)return;

    //只有站着或者趴着
    if(robot_state!=STAND && robot_state!=LIE){
        idleStart=millis();
        return;
    }

    //在步态过程中不允许无聊模式
    if(Motion_isWalking()){
        idleStart=millis();
        return;
    }




    if(millis()-lastBoredIncrease>1000)
    {
        lastBoredIncrease = millis();

        boredLevel++;
        if(boredLevel>100)
            boredLevel=100;
    }




    unsigned long idle = millis()-idleStart;//距离上次无聊时间开始计时到现在过去了多久

    //时间到达，概率触发无聊事件
    if(idle > nextBoredTime){
        int chance=random(0,10); //抽签
        if(chance<6){            //中奖
            m_boredEventFlag = true;
            waitingForExpressionFinish = true;
        }else{
            idleStart=millis();
        }


        int idleTime;
        if(boredLevel<30)
        {
            idleTime=random(7000,9000);
        }else if(boredLevel<70){
            idleTime=random(5000,7000);
        }else{
            idleTime=random(3000,5000);
        }
        nextBoredTime = idleTime;
    }
}

//外部查询：是否产生无聊事件
bool BoredSystem_isBoredEvent()
{
    return m_boredEventFlag;
}

//外部处理完表情+声音之后，调用这个清空事件标记
void BoredSystem_clearBoredEvent()
{
    m_boredEventFlag = false;
}

//设置机器人的闲聊模式
void BoredSystem_setMode()
{
    if(boredEnable == true){
        boredEnable = false;
    }else{
        boredEnable = true;
    }

    Serial.print("(切换)无聊模式:");
    Serial.println(boredEnable);
}


//返回机器人闲聊模式的状态
bool BoredSystem_getMode()
{
    return boredEnable;
}

void BoredSystem_notifyActivity()
{
    idleStart = millis();
    lastBoredIncrease = millis();
    boredLevel = 0;
    //发生活动，直接清空无聊事件
    m_boredEventFlag = false;
}

void BoredSystem_EventAction(){

    // 随机选择 0~3
    int randomFace = random(0, 4);

    FaceState selectedFace;
    VoiceSound selectedSound;


    switch(randomFace)
    {
        case 0:
            selectedFace = FACE_ID_HELPLESSNESS;
            selectedSound = SOUND_BORED_HELPLESSNESS;
            break;

        case 1:
            selectedFace = FACE_ID_EYE_BLIN_BLIN;
            selectedSound = SOUND_BORED_EYE_BLIN_BLIN;
            break;

        case 2:
            selectedFace = FACE_ID_HAPPY_CAT;
            selectedSound = SOUND_BORED_HAPPY_CAT;
            break;

        default:
            selectedFace = FACE_ID_SEEK;
            selectedSound = SOUND_BORED_SEEK;
            break;
    }


    // 尝试播放表情
    if(DisplayUI_playExternalAnim(selectedFace))
    {
        Serial.print("【无聊系统】触发随机表情: ");
        Serial.println(Bored_Voice_Name[selectedFace]);


        Voice_Insert_Queue(selectedSound);
        BoredSystem_clearBoredEvent();
    }
}