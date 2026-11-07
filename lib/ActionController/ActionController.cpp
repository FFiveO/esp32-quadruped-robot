#include "Types.h"
#include "ActionController.h"

#include "RobotConfig.h"
#include "Initialize.h"
#include "Robot_state.h"
#include "ServoDriver.h"
#include "Motion.h"
#include "VoiceManager.h"
#include "LegKinematics.h"


ActionType current_action = ACTION_NONE;       //当前正在执行什么动作
ActionStep current_step = STEP_NONE;           //当前动作执行到了哪个子步骤

static unsigned long lastTime = 0;             //上次更新步态的时间


static float STAND_OR_LIE_DURATION = 2.5f;     //机器狗站起或者躺下的持续时间

//挥手参数-------------------------------------------------------------------------------------------------
static float WAVE_BODY_TIME = 0.6f;            //坐下和站起时间
static float WAVE_HAND_TIME = 0.8f;            //握手时间

static float WAVE_LIE_BACK_KNEE = -20.0f;      //一开始趴下时后小臂的弯曲度数
static float WAVE_LIE_FRONT_HIP = 15.0f;       //一开始趴下时前大臂的弯曲度数
static float WAVE_LIE_FRONT_KNEE = 15.0f;      //一开始趴下时前小臂的弯曲度数

static float WAVE_ONE_S_HIP = 60.0f;           //第一次往前抬大臂
static float WAVE_ONE_S_KNEE = -130.0f;        //第一次往上抬小臂
static float WAVE_ONE_E_HIP = -40.0f;          //第一次抬大臂收(角度不完全回)
static float WAVE_ONE_E_KNEE = 90.0f;          //第一次抬小臂收(角度不完全回)
static float WAVE_TWO_S_HIP = 40.0f;           //第二次往前抬大臂
static float WAVE_TWO_S_KNEE = -90.0f;         //第二次往上抬小臂   
static float WAVE_TWO_E_HIP = -60.0f;          //第二次抬大臂收
static float WAVE_TWO_E_KNEE = 130.0f;         //第二次抬小臂收



//俯卧撑参数-----------------------------------------------------------------------------------------------
static float PUSH_UP_HAND_TIME = 0.6f;        //俯卧撑前腿抬起放下时间
static float PUSH_UP_BODY_TIME = 0.6f;        //俯卧撑身体趴下抬起时间
static float PUSH_UP_CHANGE_BALANCE_TIME = 0.6f;  //俯卧撑身体趴下抬起时间

static float PUSH_UP_LIE_HIP =  -40.0f;       //俯卧撑动作开始所有大腿往后角度
static float PUSH_UP_LIE_KNEE = -60.0f;       //俯卧撑动作开始所有小腿往上角度

static float PUSH_UP_RAISE_HIP_S =  30.0f;    //俯卧撑起身前大腿角度
static float PUSH_UP_RAISE_HIP_E =  -30.0f;   //俯卧撑趴下前大腿角度
static float PUSH_UP_RAISE_KNEE_S = 50.0f;    //俯卧撑起身前小腿角度
static float PUSH_UP_RAISE_KNEE_E = -50.0f;   //俯卧撑趴下前小腿角度

static float PUSH_UP_CHANGE_BALANCE = 20.0f;  //站立前调整重心(向前角度)



//撒尿参数-------------------------------------------------------------------------------------------------
static float PEE_BODY_TIME = 0.8f;        //撒尿四条腿调整重心时间花费
static float PEE_RB_TIME = 0.8f;          //撒尿右后腿往后抬时间花费
static float PEE_RBK_TIME = 0.15f;        //撒尿右后小腿前后往返时间花费

static float PEE_BODY_ANGLE = -15.0f;     //撒尿四条腿调整重心(向后角度)
static float PEE_RBK_ANGLE_S = 50.0f;     //撒尿右后小腿往后蹬角度
static float PEE_RBK_ANGLE_E = -50.0f;    //撒尿右后小腿往前收角度

//扭动参数 IK模式(单位mm)-------------------------------------------------------------------------------------------------
static float filtZ[4] = {0, 0, 0, 0};

BodyWaveAction bodyWave = {0};
const float filterAlpha = 0.12f; 



//余弦波形：0开始缓起，峰值中间，末尾缓落
static float Wave_GetLegZ(float t, float dur, float zBase, float zPeak)
{
    if(t <= 0.0f || t >= dur){
        return zBase;
    }

    float phase = t / dur;
    float s = 0.5f * ( 1.0f - cosf( phase * PI * 2.0) );
    return zBase + (zPeak - zBase) * s;
}


//子步骤分发(负责把下一步真正配置出来)
void Start_Action_Step(ActionStep step){ 
    switch(step){


        //站立
        case ENTIRE_LIE_TO_STAND:
            set_TO_STAND_START();
            break;

        //趴下
        case ENTIRE_STAND_TO_LIE:
            set_TO_LIE_START();
            break;


        //挥手
        case WAVE_LEG_START_MOVE:
            set_WAVE_ALL_LEG_START();
            break;
        case WAVE_HAND_ONE_S:
            set_WAVE_R_HANG_ONE_S();
            break;
        case WAVE_HAND_ONE_E:
            set_WAVE_R_HANG_ONE_E();
            break;
        case WAVE_HAND_TWO_S:
            set_WAVE_R_HANG_TWO_S();
            break;
        case WAVE_HAND_TWO_E:
            set_WAVE_R_HANG_TWO_E();
            break;
        case WAVE_LEG_END_MOVE:
            set_WAVE_ALL_LEG_END();
            break;

        //俯卧撑
        case PUSH_UP_LEG_START_MOVE:
            set_PUSH_UP_LEG_START_MOVE();
            break;

        case PUSH_UP_ONE_S:
        case PUSH_UP_TWO_S:
        case PUSH_UP_THREE_S:
        case PUSH_UP_FOUR_S:
            set_PUSH_UP_RAISE();
            break;


        case PUSH_UP_ONE_E:
        case PUSH_UP_TWO_E:
        case PUSH_UP_THREE_E:
        case PUSH_UP_FOUR_E:
            set_PUSH_UP_LOWER();
            break;

        case PUSH_UP_CHANGE:
            set_PUSH_UP_CHANGE();
            break;
        case PUSH_UP_LEG_END_MOVE:
            set_PUSH_UP_LEG_END_MOVE();
            break;

            
        //撒尿动作
        case PEE_LEG_START_MOVE:
            set_PEE_LEG_START_MOVE();
            break;
        case PEE_RB_LEG_RAISE:
            set_PEE_RB_LEG_RAISE();
            break;

        case PEE_RBK_ONE_S:
        case PEE_RBK_TWO_S:
        case PEE_RBK_THREE_S:
            set_PEE_RBK_BACK();
            break;

        case PEE_RBK_ONE_E:
        case PEE_RBK_TWO_E:
        case PEE_RBK_THREE_E:
            set_PEE_RBK_FORWARD();
            break;

        case PEE_LEG_END_MOVE:
            set_PEE_LEG_END_MOVE();
            break;

    }
}

//状态机（负责决定下一步是谁）
void Action_NextStep(){

    switch(current_action)
    {
        case LIE_TO_STAND:  //趴下 ---> 站立

            Serial.println("进入站立模式(当前运动模式为:IK)");
            Action_End(STAND);
            break;

        case STAND_TO_LIE:   //站立 ---> 趴下
            Action_End(LIE);
            break;

        case WAVE:
            if(current_step == WAVE_LEG_START_MOVE){
                current_step = WAVE_HAND_ONE_S;
                Start_Action_Step(current_step);

            }else if(current_step == WAVE_HAND_ONE_S){
                Voice_Insert_Queue(SOUND_LONG);
                current_step = WAVE_HAND_ONE_E;
                Start_Action_Step(current_step);

            }else if(current_step == WAVE_HAND_ONE_E){

                current_step = WAVE_HAND_TWO_S;
                Start_Action_Step(current_step);

            }else if(current_step == WAVE_HAND_TWO_S){

                current_step = WAVE_HAND_TWO_E;
                Start_Action_Step(current_step);

            }else if(current_step == WAVE_HAND_TWO_E){

                current_step = WAVE_LEG_END_MOVE;
                Start_Action_Step(current_step);

            }else if(current_step == WAVE_LEG_END_MOVE){
                Action_End(STAND);
            }
            break;
        case PUSH_UP:
            if(current_step == PUSH_UP_LEG_START_MOVE){
                current_step = PUSH_UP_ONE_S;
                Start_Action_Step(current_step);

            }else if(current_step == PUSH_UP_ONE_S){

                current_step = PUSH_UP_ONE_E;
                Start_Action_Step(current_step);

            }else if(current_step == PUSH_UP_ONE_E){
                Voice_Insert_Queue(SOUND_CURIOUS);
                current_step = PUSH_UP_TWO_S;
                Start_Action_Step(current_step);

            }else if(current_step == PUSH_UP_TWO_S){

                current_step = PUSH_UP_TWO_E;
                Start_Action_Step(current_step);

            }else if(current_step == PUSH_UP_TWO_E){

                current_step = PUSH_UP_THREE_S;
                Start_Action_Step(current_step);

            }else if(current_step == PUSH_UP_THREE_S){
                current_step = PUSH_UP_THREE_E;
                Start_Action_Step(current_step);

            }else if(current_step == PUSH_UP_THREE_E){
                Voice_Insert_Queue(SOUND_CURIOUS);
                current_step = PUSH_UP_FOUR_S;
                Start_Action_Step(current_step);

            }else if(current_step == PUSH_UP_FOUR_S){

                current_step = PUSH_UP_FOUR_E;
                Start_Action_Step(current_step);

            }else if(current_step == PUSH_UP_FOUR_E){

                current_step = PUSH_UP_CHANGE;
                Start_Action_Step(current_step);

            }else if(current_step == PUSH_UP_CHANGE){

                current_step = PUSH_UP_LEG_END_MOVE;
                Start_Action_Step(current_step);

            }else if(current_step == PUSH_UP_LEG_END_MOVE){
                Action_End(STAND);
            }
            break;
        case PEE:
            if(current_step == PEE_LEG_START_MOVE){
                current_step = PEE_RB_LEG_RAISE;
                Start_Action_Step(current_step);
            }else if(current_step == PEE_RB_LEG_RAISE){
                Voice_Insert_Queue(SOUND_HAPPY);
                current_step = PEE_RBK_ONE_S;
                Start_Action_Step(current_step);
            }else if(current_step == PEE_RBK_ONE_S){
                current_step = PEE_RBK_ONE_E;
                Start_Action_Step(current_step);
            }else if(current_step == PEE_RBK_ONE_E){
                Voice_Insert_Queue(SOUND_HAPPY);
                current_step = PEE_RBK_TWO_S;
                Start_Action_Step(current_step);
            }else if(current_step == PEE_RBK_TWO_S){
                current_step = PEE_RBK_TWO_E;
                Start_Action_Step(current_step);
            }else if(current_step == PEE_RBK_TWO_E){
                Voice_Insert_Queue(SOUND_HAPPY);
                current_step = PEE_RBK_THREE_S;
                Start_Action_Step(current_step);
            }else if(current_step == PEE_RBK_THREE_S){
                current_step = PEE_RBK_THREE_E;
                Start_Action_Step(current_step);
            }else if(current_step == PEE_RBK_THREE_E){
                current_step = PEE_LEG_END_MOVE;
                Start_Action_Step(current_step);
            }else if(current_step == PEE_LEG_END_MOVE){
                Action_End(STAND);
            }
    }
}

//动作入口
void Action_control(String action_type){ 
    action_type.trim();

    if(action_type == "wavebody"){
        Start_BodyWave();
        Serial.println("启动机身波浪动作");
        return ;
    }

    
    if(action_type == "stand" || action_type == "STAND"){

        current_action = LIE_TO_STAND;
        current_step = ENTIRE_LIE_TO_STAND;

    }else if(action_type == "lie" || action_type == "LIE"){

        current_action = STAND_TO_LIE;
        current_step = ENTIRE_STAND_TO_LIE;

    }else if(action_type == "wave"){

        current_action = WAVE;
        current_step = WAVE_LEG_START_MOVE;

    }else if(action_type == "pushup") {

        current_action = PUSH_UP;
        current_step = PUSH_UP_LEG_START_MOVE;

    }else if(action_type == "pee"){

        current_action = PEE;
        current_step = PEE_LEG_START_MOVE;

    }else{
        Serial.print("未知静态动作: ");
        Serial.println(action_type);
        return;
    }

    for(int i=0;i<4;i++){
        leg[i].control_mode = ANGLE_MODE;
    }

    Start_Action_Step(current_step); //设置开头动作子步骤的运动角度区间
    lastTime = millis();
    robot_state = TRANSITION; //进入到更新状态
}

//运动执行
void Action_update(){

    //1. 独立波浪动作
    if(bodyWave.active)
    {
        Update_BodyWave();
        return;
    }

    // 2. 当前没有普通静态动作
    if(current_action == ACTION_NONE)
    {
        return;
    }

    //3. 当前子步骤结束
    if(Leg_isAllFinished() == true){     //当动作执行结束进入状态
        Serial.println("结束啦");
        Action_NextStep();
        return ;
    }

    //4. 当前子步骤继续执行

    unsigned long now = millis();//求取时间差dt

    float dt = (now - lastTime) / 1000.0f;//距离上次前进的时间差
    lastTime = now;

    if (dt < 0.0f || dt > 0.2f) {
      dt = 0.01f;
    }


    //根据dt更新每条腿当前角度,并转动(ps： right_to_left函数只是在初始一个动作的target_angle时才需要)
    for(int i=3 ; i>=0 ; i--){
        if(leg[i].control_mode == ANGLE_MODE){
            if(leg[i].angle_action.running == false)continue;
            //leg[i].control_mode != ANGLE_MODE || 
            bool move_over = true;//认为第i条腿这次更新完就到了目标角度(大腿小腿)

            leg[i].angle_action.phase += dt / leg[i].angle_action.duration;  //腿运动阶段改变

            if(leg[i].angle_action.phase >= 1.0f)
            {
                leg[i].angle_action.phase = 1.0f;
            }

            if(abs(leg[i].angle_action.target_hip - leg[i].angle_action.current_hip) > 1.0f){//如果大腿还没到
                leg[i].angle_action.current_hip = leg[i].angle_action.start_hip + (leg[i].angle_action.target_hip - leg[i].angle_action.start_hip) * leg[i].angle_action.phase;
                ServoDriver_writeServo(leg[i].Hip, leg[i].angle_action.current_hip);
                move_over = false;
            }

            if(abs(leg[i].angle_action.target_knee - leg[i].angle_action.current_knee) > 1.0f){//如果小腿没到
                leg[i].angle_action.current_knee = leg[i].angle_action.start_knee + (leg[i].angle_action.target_knee - leg[i].angle_action.start_knee) * leg[i].angle_action.phase;
                ServoDriver_writeServo(leg[i].Knee, leg[i].angle_action.current_knee);
                move_over = false;
            }


            if(move_over)
            {
                leg[i].angle_action.running = false;
                leg[i].angle_action.current_hip = leg[i].angle_action.target_hip;
                leg[i].angle_action.current_knee = leg[i].angle_action.target_knee;
            }
        
        }
    }

    
}

//更新波浪动作
void Update_BodyWave(){
        unsigned long nowMs = millis();

        float elapsedTotal = (nowMs - bodyWave.tStartMs) / 1000.0f;

        // 波浪动作结束
        if(elapsedTotal >= bodyWave.durationTotal)
        {
            bodyWave.active = false;

            //四条腿强制回到站立位置
            for(int i=0;i<4;i++)
            {
                float xNow = STAND_X;

                if(!leg[i].isFront){
                    xNow -= Back_Leg_runing_IK_offset;
                }

                filtZ[i] = bodyWave.zBase;

                Leg_IK_move(leg[i], xNow, bodyWave.zBase);

            }

            Action_End(STAND);
            Serial.println("机身波浪动作结束，恢复STAND状态");

            return;
        }


        // 波浪动作正常运行
        for(int i=0;i<4;i++)
        {
            float tLeg = elapsedTotal - bodyWave.legDelay[i];
            float zRaw = Wave_GetLegZ(tLeg, bodyWave.legMoveDur[i], bodyWave.zBase, bodyWave.zPeak);

            //低通滤波
            filtZ[i] = filtZ[i] +filterAlpha *(zRaw - filtZ[i]);

            float xNow = STAND_X;

            if(!leg[i].isFront){
                xNow -= Back_Leg_runing_IK_offset;
            }


            Leg_IK_move(leg[i],xNow,filtZ[i]);
        }
}

//状态机的“完成检测器”
bool Leg_isAllFinished(){
    for(int i=0 ; i<4 ; i++){

      if(leg[i].control_mode == ANGLE_MODE){
          if(leg[i].angle_action.running == true) return false;

      }else if(leg[i].control_mode == IK_MODE){

          if(leg[i].ik_action.running == true) return false;

      }
    }
    return true;
}

//动作结束重置动作信息
void Action_End(RobotState flag)
{
    current_action = ACTION_NONE;
    current_step = STEP_NONE;

    for(int i = 0; i < 4; i++)
    {
        //结束所有静态动作运行状态
        leg[i].angle_action.running = false;
        leg[i].ik_action.running = false;

        //站立状态交回IK控制
        if(flag == STAND)
        {
            leg[i].control_mode = IK_MODE;
        }
        //趴下状态继续保持角度控制
        else if(flag == LIE)
        {
            leg[i].control_mode = ANGLE_MODE;
        }
    }

    robot_state = flag;
}


//腿角度镜像
float Right_to_Left(float angle){
  return angle * -1.0;
}









//站立、趴下角度设置-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//设置角度模式的角度为 lie_to_stand的角度
void set_TO_STAND_START(){
        set_ONE_LEG_ANGLE(3,     Action_getStandHip(3) + Stand_to_Lie_Front_Hip_Offset,                    Action_getStandHip(3),       Action_getStandKnee(3) + Stand_to_Lie_Front_Knee_Offset,                    Action_getStandKnee(3),         STAND_OR_LIE_DURATION);
        set_ONE_LEG_ANGLE(2,     Action_getStandHip(2) + Right_to_Left(Stand_to_Lie_Front_Hip_Offset),     Action_getStandHip(2),       Action_getStandKnee(2) + Right_to_Left(Stand_to_Lie_Front_Knee_Offset),     Action_getStandKnee(2),         STAND_OR_LIE_DURATION);
        set_ONE_LEG_ANGLE(1,     Action_getStandHip(1) + Right_to_Left(Stand_to_Lie_Back_Hip_Offset),      Action_getStandHip(1),       Action_getStandKnee(1) + Right_to_Left(Stand_to_Lie_Back_Knee_Offset),      Action_getStandKnee(1),         STAND_OR_LIE_DURATION);
        set_ONE_LEG_ANGLE(0,     Action_getStandHip(0) + Stand_to_Lie_Back_Hip_Offset,                     Action_getStandHip(0),       Action_getStandKnee(0) + Stand_to_Lie_Back_Knee_Offset,                     Action_getStandKnee(0),         STAND_OR_LIE_DURATION);
}
//设置角度模式的角度为 stand_to_lie的角度
void set_TO_LIE_START(){
        set_ONE_LEG_ANGLE(3,     Action_getStandHip(3),     Action_getStandHip(3) + Stand_to_Lie_Front_Hip_Offset,                      Action_getStandKnee(3),         Action_getStandKnee(3) + Stand_to_Lie_Front_Knee_Offset,                         STAND_OR_LIE_DURATION);
        set_ONE_LEG_ANGLE(2,     Action_getStandHip(2),     Action_getStandHip(2) + Right_to_Left(Stand_to_Lie_Front_Hip_Offset),       Action_getStandKnee(2),         Action_getStandKnee(2) + Right_to_Left(Stand_to_Lie_Front_Knee_Offset),          STAND_OR_LIE_DURATION);
        set_ONE_LEG_ANGLE(1,     Action_getStandHip(1),     Action_getStandHip(1) + Right_to_Left(Stand_to_Lie_Back_Hip_Offset),        Action_getStandKnee(1),         Action_getStandKnee(1) + Right_to_Left(Stand_to_Lie_Back_Knee_Offset - 5),       STAND_OR_LIE_DURATION);
        set_ONE_LEG_ANGLE(0,     Action_getStandHip(0),     Action_getStandHip(0) + Stand_to_Lie_Back_Hip_Offset,                       Action_getStandKnee(0),         Action_getStandKnee(0) + Stand_to_Lie_Back_Knee_Offset - 5,                      STAND_OR_LIE_DURATION);
}



//挥手动作序列角度设置----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//设置开始挥手的坐下
void set_WAVE_ALL_LEG_START(){
        set_ONE_LEG_ANGLE(3,     Action_getStandHip(3),     Action_getStandHip(3) + WAVE_LIE_FRONT_HIP,                    Action_getStandKnee(3),     Action_getStandKnee(3) + WAVE_LIE_FRONT_KNEE,                    WAVE_BODY_TIME);
        set_ONE_LEG_ANGLE(2,     Action_getStandHip(2),     Action_getStandHip(2) + Right_to_Left(WAVE_LIE_FRONT_HIP),     Action_getStandKnee(2),     Action_getStandKnee(2) + Right_to_Left(WAVE_LIE_FRONT_KNEE),     WAVE_BODY_TIME);
        set_ONE_LEG_ANGLE(1,     Action_getStandHip(1),     Action_getStandHip(1),                                         Action_getStandKnee(1),     Action_getStandKnee(1) + Right_to_Left(WAVE_LIE_BACK_KNEE - 10),      WAVE_BODY_TIME);
        set_ONE_LEG_ANGLE(0,     Action_getStandHip(0),     Action_getStandHip(0),                                         Action_getStandKnee(0),     Action_getStandKnee(0) + WAVE_LIE_BACK_KNEE,                     WAVE_BODY_TIME);
}
//设置坐下后第一次挥手
void set_WAVE_R_HANG_ONE_S(){
        set_ONE_LEG_ANGLE(3,   leg[3].angle_action.target_hip,   leg[3].angle_action.target_hip + WAVE_ONE_S_HIP,   leg[3].angle_action.target_knee,   leg[3].angle_action.target_knee + WAVE_ONE_S_KNEE,   WAVE_HAND_TIME);
}
//设置第一次挥手的收回
void set_WAVE_R_HANG_ONE_E(){
        set_ONE_LEG_ANGLE(3,   leg[3].angle_action.target_hip,   leg[3].angle_action.target_hip + WAVE_ONE_E_HIP,   leg[3].angle_action.target_knee,   leg[3].angle_action.target_knee + WAVE_ONE_E_KNEE,   WAVE_HAND_TIME);
}
//设置第二次挥手
void set_WAVE_R_HANG_TWO_S(){
        set_ONE_LEG_ANGLE(3,   leg[3].angle_action.target_hip,   leg[3].angle_action.target_hip + WAVE_TWO_S_HIP,   leg[3].angle_action.target_knee,   leg[3].angle_action.target_knee + WAVE_TWO_S_KNEE,   WAVE_HAND_TIME);
}
//设置第二次挥手的收回
void set_WAVE_R_HANG_TWO_E(){
        set_ONE_LEG_ANGLE(3,   leg[3].angle_action.target_hip,   leg[3].angle_action.target_hip + WAVE_TWO_E_HIP,   leg[3].angle_action.target_knee,   leg[3].angle_action.target_knee + WAVE_TWO_E_KNEE,   WAVE_HAND_TIME);
}
//设置挥完手后的起身
void set_WAVE_ALL_LEG_END(){
        set_ONE_LEG_ANGLE(3,     leg[3].angle_action.target_hip,     Action_getStandHip(3),     leg[3].angle_action.target_knee,     Action_getStandKnee(3), WAVE_BODY_TIME);
        set_ONE_LEG_ANGLE(2,     leg[2].angle_action.target_hip,     Action_getStandHip(2),     leg[2].angle_action.target_knee,     Action_getStandKnee(2), WAVE_BODY_TIME);
        set_ONE_LEG_ANGLE(1,     leg[1].angle_action.target_hip,     Action_getStandHip(1),     leg[1].angle_action.target_knee,     Action_getStandKnee(1), WAVE_BODY_TIME);
        set_ONE_LEG_ANGLE(0,     leg[0].angle_action.target_hip,     Action_getStandHip(0),     leg[0].angle_action.target_knee,     Action_getStandKnee(0), WAVE_BODY_TIME);
}


//俯卧撑动作序列角度设置----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//设置俯卧撑开始趴下
void set_PUSH_UP_LEG_START_MOVE(){
        set_ONE_LEG_ANGLE(3,   Action_getStandHip(3),   Action_getStandHip(3) + PUSH_UP_LIE_HIP,                  Action_getStandKnee(3),   Action_getStandKnee(3) + PUSH_UP_LIE_KNEE,                  PUSH_UP_BODY_TIME);
        set_ONE_LEG_ANGLE(2,   Action_getStandHip(2),   Action_getStandHip(2) + Right_to_Left(PUSH_UP_LIE_HIP),   Action_getStandKnee(2),   Action_getStandKnee(2) + Right_to_Left(PUSH_UP_LIE_KNEE),   PUSH_UP_BODY_TIME);
        set_ONE_LEG_ANGLE(1,   Action_getStandHip(1),   Action_getStandHip(1) + Right_to_Left(PUSH_UP_LIE_HIP),   Action_getStandKnee(1),   Action_getStandKnee(1) + Right_to_Left(PUSH_UP_LIE_KNEE),   PUSH_UP_BODY_TIME);
        set_ONE_LEG_ANGLE(0,   Action_getStandHip(0),   Action_getStandHip(0) + PUSH_UP_LIE_HIP,                  Action_getStandKnee(0),   Action_getStandKnee(0) + PUSH_UP_LIE_KNEE,                  PUSH_UP_BODY_TIME);
}
//抬起
void set_PUSH_UP_RAISE(){
        set_ONE_LEG_ANGLE(3,   leg[3].angle_action.target_hip,   leg[3].angle_action.target_hip + PUSH_UP_RAISE_HIP_S,                  leg[3].angle_action.target_knee,   leg[3].angle_action.target_knee + PUSH_UP_RAISE_KNEE_S,                      PUSH_UP_HAND_TIME);
        set_ONE_LEG_ANGLE(2,   leg[2].angle_action.target_hip,   leg[2].angle_action.target_hip + Right_to_Left(PUSH_UP_RAISE_HIP_S),   leg[2].angle_action.target_knee,   leg[2].angle_action.target_knee + Right_to_Left(PUSH_UP_RAISE_KNEE_S),       PUSH_UP_HAND_TIME);
}
//放下
void set_PUSH_UP_LOWER(){
        set_ONE_LEG_ANGLE(3,   leg[3].angle_action.target_hip,   leg[3].angle_action.target_hip + PUSH_UP_RAISE_HIP_E,                  leg[3].angle_action.target_knee,   leg[3].angle_action.target_knee + PUSH_UP_RAISE_KNEE_E,                    PUSH_UP_HAND_TIME);
        set_ONE_LEG_ANGLE(2,   leg[2].angle_action.target_hip,   leg[2].angle_action.target_hip + Right_to_Left(PUSH_UP_RAISE_HIP_E),     leg[2].angle_action.target_knee,   leg[2].angle_action.target_knee + Right_to_Left(PUSH_UP_RAISE_KNEE_E),       PUSH_UP_HAND_TIME);
}
//调整重心
void set_PUSH_UP_CHANGE(){
        set_ONE_LEG_ANGLE(3,   leg[3].angle_action.target_hip,   leg[3].angle_action.target_hip + PUSH_UP_CHANGE_BALANCE,                  leg[3].angle_action.target_knee, leg[3].angle_action.target_knee,   PUSH_UP_CHANGE_BALANCE_TIME);
        set_ONE_LEG_ANGLE(2,   leg[2].angle_action.target_hip,   leg[2].angle_action.target_hip + Right_to_Left(PUSH_UP_CHANGE_BALANCE),   leg[2].angle_action.target_knee, leg[2].angle_action.target_knee,   PUSH_UP_CHANGE_BALANCE_TIME);
        set_ONE_LEG_ANGLE(1,   leg[1].angle_action.target_hip,   leg[1].angle_action.target_hip + Right_to_Left(PUSH_UP_CHANGE_BALANCE),   leg[1].angle_action.target_knee, leg[1].angle_action.target_knee,   PUSH_UP_CHANGE_BALANCE_TIME);
        set_ONE_LEG_ANGLE(0,   leg[0].angle_action.target_hip,   leg[0].angle_action.target_hip + PUSH_UP_CHANGE_BALANCE,                  leg[0].angle_action.target_knee, leg[0].angle_action.target_knee,   PUSH_UP_CHANGE_BALANCE_TIME);
}
//起身
void set_PUSH_UP_LEG_END_MOVE(){
        set_ONE_LEG_ANGLE(3,   leg[3].angle_action.target_hip,   Action_getStandHip(3),   leg[3].angle_action.target_knee, Action_getStandKnee(3),   PUSH_UP_BODY_TIME);
        set_ONE_LEG_ANGLE(2,   leg[2].angle_action.target_hip,   Action_getStandHip(2),   leg[2].angle_action.target_knee, Action_getStandKnee(2),   PUSH_UP_BODY_TIME);
        set_ONE_LEG_ANGLE(1,   leg[1].angle_action.target_hip,   Action_getStandHip(1),   leg[1].angle_action.target_knee, Action_getStandKnee(1),   PUSH_UP_BODY_TIME);
        set_ONE_LEG_ANGLE(0,   leg[0].angle_action.target_hip,   Action_getStandHip(0),   leg[0].angle_action.target_knee, Action_getStandKnee(0),   PUSH_UP_BODY_TIME);
}


//撒尿动作序列角度设置----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//设置撒尿动作开始重心
void set_PEE_LEG_START_MOVE(){
        set_ONE_LEG_ANGLE(3,   Action_getStandHip(3),   Action_getStandHip(3) + PEE_BODY_ANGLE,                  Action_getStandKnee(3),   Action_getStandKnee(3),                  PEE_BODY_TIME);
        set_ONE_LEG_ANGLE(2,   Action_getStandHip(2),   Action_getStandHip(2) + Right_to_Left(PEE_BODY_ANGLE),   Action_getStandKnee(2),   Action_getStandKnee(2),                  PEE_BODY_TIME);
        set_ONE_LEG_ANGLE(1,   Action_getStandHip(1),   Action_getStandHip(1) + Right_to_Left(PEE_BODY_ANGLE),   Action_getStandKnee(1),   Action_getStandKnee(1),                  PEE_BODY_TIME);
        set_ONE_LEG_ANGLE(0,   Action_getStandHip(0),   Action_getStandHip(0) + PEE_BODY_ANGLE,                  Action_getStandKnee(0),   Action_getStandKnee(0),                  PEE_BODY_TIME);
}
//右后小腿后伸
void set_PEE_RB_LEG_RAISE(){
        set_ONE_LEG_ANGLE(0,   leg[0].angle_action.target_hip,   leg[0].angle_action.target_hip + (-40),                 leg[0].angle_action.target_knee,   leg[0].angle_action.target_knee + (-70),                  PEE_RB_TIME);
}
//第一次往后_开始
void set_PEE_RBK_BACK(){
        set_ONE_LEG_ANGLE(0,   leg[0].angle_action.target_hip,   leg[0].angle_action.target_hip,                 leg[0].angle_action.target_knee,   leg[0].angle_action.target_knee + PEE_RBK_ANGLE_S,                PEE_RBK_TIME);
}
//第一次往后_结束
void set_PEE_RBK_FORWARD(){
        set_ONE_LEG_ANGLE(0,   leg[0].angle_action.target_hip,   leg[0].angle_action.target_hip,                 leg[0].angle_action.target_knee,   leg[0].angle_action.target_knee + PEE_RBK_ANGLE_E,                PEE_RBK_TIME);
}
//起身
void set_PEE_LEG_END_MOVE(){
        set_ONE_LEG_ANGLE(3,   leg[3].angle_action.target_hip,     Action_getStandHip(3),      leg[3].angle_action.target_knee, Action_getStandKnee(3),     PEE_BODY_TIME);
        set_ONE_LEG_ANGLE(2,   leg[2].angle_action.target_hip,     Action_getStandHip(2),      leg[2].angle_action.target_knee, Action_getStandKnee(2),     PEE_BODY_TIME);
        set_ONE_LEG_ANGLE(1,   leg[1].angle_action.target_hip,     Action_getStandHip(1),      leg[1].angle_action.target_knee, Action_getStandKnee(1),     PEE_BODY_TIME);
        set_ONE_LEG_ANGLE(0,   leg[0].angle_action.target_hip,     Action_getStandHip(0),      leg[0].angle_action.target_knee, Action_getStandKnee(0),     PEE_BODY_TIME);
}


//角度设置核心工具---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//设置一条腿转动目标和时间的工具
void set_ONE_LEG_ANGLE(int LegId, float H_S, float H_E, float K_S, float K_E, float duration){
      leg[LegId].angle_action.start_hip = H_S;
      leg[LegId].angle_action.current_hip = H_S;
      leg[LegId].angle_action.target_hip = H_E;

      leg[LegId].angle_action.start_knee = K_S;
      leg[LegId].angle_action.current_knee = K_S;
      leg[LegId].angle_action.target_knee = K_E;

      leg[LegId].angle_action.phase = 0;
      leg[LegId].angle_action.duration = duration;
      
      leg[LegId].angle_action.running = true;

      Serial.print("SET LEG ");
      Serial.print(LegId);
      Serial.print(" hip:");
      Serial.print(H_S);
      Serial.print("->");
      Serial.print(H_E);
      Serial.print(" knee:");
      Serial.print(K_S);
      Serial.print("->");
      Serial.println(K_E);
}


//获取legID条腿标准大腿关节角度
float Action_getStandHip(int legID)
{
    return Standard_Leg_Stand_Angle[0][legID];
}

//获取legID条腿标准小腿关节角度
float Action_getStandKnee(int legID)
{
    return Standard_Leg_Stand_Angle[1][legID];
}


//启动波浪：时序：2左前(0.0s) →1左后(0.2s) →0右后(0.6s) →3右前(0.8s)
void Start_BodyWave(void)
{
    bodyWave.active = true;                //开启波浪运动
    bodyWave.tStartMs = millis();

    //腿编号：0右后，1左后，2左前，3右前
    //【关键】缩小delay间隔，波形互相重叠！！

    float legDur = 1.5f;   //单腿抬起+落下总时长
    float Leg_delay = legDur - 0.8f;
    bodyWave.durationTotal = legDur + Leg_delay * 3 + 0.3f;   //整套总时间
    
    bodyWave.legDelay[2] = 0.0f;            //左前腿最开始动
    bodyWave.legDelay[1] = Leg_delay;       //左前腿运行后第一个Leg_delay开始运动
    bodyWave.legDelay[0] = Leg_delay * 2;   //左前腿运行后第二个Leg_delay开始运动
    bodyWave.legDelay[3] = Leg_delay * 3;   //左前腿运行后第三个Leg_delay开始运动

    for(int i=0;i<4;i++){
        bodyWave.legMoveDur[i] = legDur;
    }

    bodyWave.zBase = STAND_Z;               //基准Z
    bodyWave.zPeak = STAND_Z + 50.0f;       //踮脚最高Z

    for(int i=0;i<4;i++){
        filtZ[i] = bodyWave.zBase;
        leg[i].control_mode = IK_MODE;
        leg[i].ik_action.running = false;
    }
    robot_state = TRANSITION;
}
