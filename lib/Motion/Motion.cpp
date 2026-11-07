#include "Motion.h"

#include "RobotConfig.h"
#include "LegKinematics.h"
#include "ServoDriver.h"
#include "Test_Leg.h"
#include "Initialize.h"
#include "ActionController.h"
#include "Robot_state.h"
#include "BalanceController.h"
#include "IMU.h" 


static bool isWalking = false;                 //是否在trot步态
static bool isMarching = false;                //是否在踏步

//当前使用的步态
static GaitType currentGait = GAIT_TROT;


static unsigned long lastTime = 0;             //计算步态时间
static unsigned long lastControlTime = 0;      //用来判断网页有没有断开。


//影响步态轨迹的参数-----------------------------------------------------------------------------------------------------------------------

static float cycleTime = DEFAULT_CYCLE_TIME;   //一次步态周期的时间(一条腿完成摆动相 + 支撑相)
static float liftHeight = DEFAULT_LIFT_HEIGHT; //抬腿的高度
static float stepLength = 0.0f;                //步长
static float turnRadius = 0.0f;                //旋转角度
static float currentPhase = 0.0f;              //当前执行的阶段(四条腿的总管理周期)

float Back_Leg_runing_IK_offset = 6.0f; //后腿ik的偏移量(后腿比前腿靠后)

static float SUPPORT_PRELOAD_Z = 3.0f;// 支撑腿预加载补偿(mm) 单腿支撑时，让支撑腿稍微伸长，提高稳定性



//速度缓慢变动
static float targetStepLength = 0;
static float currentStepLength = 0;

static float targetCycleTime = DEFAULT_CYCLE_TIME;
static float currentCycleTime = DEFAULT_CYCLE_TIME;


//网页实时控制参数-------------------------------------------------------------------------------------------------------------------------

static float joystickX = 0.0f;                 // 摇杆左右:  -1.0 ~ 1.0，网页约定：x>0 左转，x<0 右转
static float joystickY = 0.0f;                 // 摇杆前后:  -1.0 ~ 1.0，网页约定：y>0 前进，y<0 后退

static float speedScale = 0.75f;               // 速度滑杆:  0.0 ~ 1.0
float bodyHeightOffset = 0.0f;                 // 机身高度:      mm，正数升高，负数降低
float bodyPitchOffset = 0.0f;                  // 机身左右倾斜:   mm，正数前倾，负数后倾
float bodyRollOffset = 0.0f;                   // 机身前后倾斜:   mm，正数左倾，负数右倾
float liftScale = 1.0f;                        // 抬腿高度倍率:   0.5 ~ 1.5

static const float JOYSTICK_DEAD_ZONE = 0.15f;                    //前后
static const float JOYSTICK_TURN_DEAD_ZONE = 0.30f;               //左右
static const unsigned long CONTROL_TIMEOUT_MS = 350;              //网页最多允许不更新时间,超出为断开连接


//----------------------------------------------------------------------------------------------------------------------------------------
static float filtCgX = 0.0f;   //新增：重心偏移低通滤波
static float filtCgY = 0.0f;   //【新增】Y前后重心滤波
const float CG_ALPHA = 0.06f;  //滤波系数，越小越顺滑，响应越慢
const float PRE_SHIFT_TIME = 0.12f; //预移重心窗口，腿还没抬起，先挪重心
const float CG_SHIFT_MM_X = 8.0f;  //左右偏移最大mm
const float CG_SHIFT_MM_Y = 7.0f;  //前后偏移最大mm，7起步，根据你的机器调

//网页摇杆范围限制
static float clampUnit(float value) {
  return constrain(value, -1.0f, 1.0f);
}


//返回当前腿(在高度滑杆实时变化下)的 "z" 值
static float Motion_getZ_Change_Height() {
  // STAND_Z 是负数。正数高度偏移让腿更伸展，机身更高；负数让机身降低。
  return STAND_Z - bodyHeightOffset;
}


//返回当前腿(左右倾滑杆变化下)(主动让机器人歪)的 "z" 偏移值
static float Motion_getBodyZOffsetForLeg(int legId) {
  float offset = 0.0f;                      

  offset += leg[legId].isFront ? -bodyRollOffset : bodyRollOffset;  // 前倾：前腿略压低, 后腿略抬高；如果实物方向反了, 只需要把这里的正负号对调
  offset += leg[legId].isLeft ? bodyPitchOffset : -bodyPitchOffset;  // 左倾：左腿略压低, 右腿略抬高；如果实物方向反了, 只需要把这里的正负号对调

  return offset;
}


//让腿移动到当前的x,z的位置(这时z的高度是实时变化的)
static void Motion_applyStandingPose() {

    bool stanceState[4] = {true, true, true, true};//站立时四条腿都是支撑腿

    BalanceOffset balance = BalanceController_update(stanceState);

    for (int i = 3; i >= 0; i--) {

      float standX = STAND_X;

      if (!leg[i].isFront) {
          standX -= Back_Leg_runing_IK_offset; // 后腿站立位置向后移动
      }

      float standZ = Motion_getZ_Change_Height() + Motion_getBodyZOffsetForLeg(i);
      
      Leg_IK_move(leg[i], standX + balance.x[i], standZ + balance.z[i]);
    }
}

// 根据当前步态，计算某条腿自己的局部phase
static float Motion_getLegPhase(int legId, float globalPhase)
{
    // TROT
    if(currentGait == GAIT_TROT)
    {
        float phaseOffset = 0.0f;

        if(legId == 0 || legId == 2)
        {
            phaseOffset = 0.5f;
        }

        float legPhase = globalPhase + phaseOffset;

        while(legPhase >= 1.0f)
        {
            legPhase -= 1.0f;
        }

        return legPhase;
    }


    // SWING
    if(currentGait == GAIT_WALK)
    {
        /*
            腿号：
            0 = 右后
            1 = 左后
            2 = 左前
            3 = 右前

            这里的数字直接表示：
            这条腿在整个周期的什么时候开始抬腿。
        */

        // GAIT_WALK 四拍静步态 swingStart
        static const float walkStart[4] =
        {
            0.50f,  //0 右后
            0.25f,  //1 左后
            0.00f,  //2 左前
            0.75f   //3 右前
        };

        float legPhase =
            globalPhase - walkStart[legId];

        while(legPhase < 0.0f)
        {
            legPhase += 1.0f;
        }

        while(legPhase >= 1.0f)
        {
            legPhase -= 1.0f;
        }

        return legPhase;
    }


    return globalPhase;
}

//根据不同步态获取不同的摆动相时间占比
static float Motion_getBDX_Time()
{
    switch(currentGait)
    {
        case GAIT_TROT:
            return 0.16f;


        case GAIT_WALK:
            return 0.25f;


        default:
            return 0.10f;
    }
}

//计算phase阶段下legId号腿的目标x,z以及是否为支撑腿
static void Motion_calculateStep(int legId, float phase, float &outX, float &outZ, bool &isStance) {
  
  float legPhase = Motion_getLegPhase(legId, phase);

  float BDX_Time = Motion_getBDX_Time();

  float baseX = 0.0f;

  if (stepLength != 0.0f) {
    if (legPhase < BDX_Time) {                  //摆动相的腿
      float angle = (legPhase / BDX_Time) * PI;
      baseX = (stepLength / 2.0f) * (1.0f - cos(angle)) - (stepLength / 2.0f);
    } else {                                    //支撑相的腿
      float stanceProgress = (legPhase - BDX_Time) / (1.0f - BDX_Time);
      baseX = (stepLength / 2.0f) - (stepLength * stanceProgress);
    }
  }

  float turnX = 0.0f;

  // 原地/边走边转。turnRadius 会被摇杆 x 实时改变。
  if (turnRadius != 0.0f) {
    float turnDir = (turnRadius > 0.0f) ? 1.0f : -1.0f;//旋转方向

    // 左侧腿：leg[1] 左后，leg[2] 左前
    if (legId == 1 || legId == 2) {
      turnX = turnDir * TURN_SPEED * 0.5f;
    }
    // 右侧腿：leg[0] 右后，leg[3] 右前
    else {
      turnX = -turnDir * TURN_SPEED * 0.5f;
    }

    if (legPhase < BDX_Time) {
      float angle = (legPhase / BDX_Time) * PI;     
      turnX = turnX * sin(angle);    //让旋转的x, 根据摆动相的进度, 利用三角函数, 进行平滑处理
    }
  }

  outX = baseX + turnX; //最终的范围x输出范围

  isStance = (legPhase >= BDX_Time);


  if (!leg[legId].isFront) {//后腿x偏差
      outX -= Back_Leg_runing_IK_offset;
  }

  outZ = Motion_getZ_Change_Height() + Motion_getBodyZOffsetForLeg(legId);  //标准腿在滑杆参数影响下站立的值

  // 抬腿
  if (legPhase < BDX_Time) {
    float angle = (legPhase / BDX_Time) * PI;
    outZ = outZ + liftHeight * sin(angle);   //一样的三角函数平滑处理
  }

  //改成变量，步态区分
  float supportPreload = SUPPORT_PRELOAD_Z;
  if(currentGait == GAIT_WALK)
  {
      supportPreload = 6.0f; //walk单腿抬起，支撑腿多往下预压6mm，消除间隙
  }

  if(isStance){
      outZ -= supportPreload;
  }
}

//控制机器人步态更新运动------核心!!!
void Motion_update() {

  if(isTestMode == true)return; //测试模式,不允许进行运动循环



  float smooth = 0.03f;

  currentStepLength += (targetStepLength-currentStepLength)*smooth;
  currentCycleTime += (targetCycleTime-currentCycleTime)*smooth;


  stepLength = currentStepLength;
  cycleTime = currentCycleTime;



  // 只有网页摇杆控制时才需要连接超时保护, 原地踏步 isMarching=true，不受网页心跳超时影响
  if (isWalking && !isMarching  && millis() - lastControlTime > CONTROL_TIMEOUT_MS) {  // 网页断开、手机熄屏或网络卡住时，自动停止，避免机器人继续跑。
    Motion_stop();
    return;
  }


  if (!isWalking  && robot_state == STAND) {//不在前进或者控制状态，就算不在前进状态也要实时更新lastTime,为了保证下一次网页控制器发出运动指令时的 lastTime - now 的值不会夸张,导致机器人动作夸张
    lastTime = millis();
    Motion_applyStandingPose();
    return;
  }


  //如果不是isWalking=true的状态机器狗就会直接return,因此不会到达下面的代码
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;//距离上次前进的时间差
  lastTime = now;

  if (dt < 0.0f || dt > 0.2f) {
    dt = 0.01f;
  }

  currentPhase += dt / cycleTime;

  while (currentPhase >= 1.0f){
      currentPhase -= 1.0f;
  }

    // ------------------------------------------------------
  // 检测某一组腿是否刚刚完成落脚
  //
  // 两组对角腿的落脚点：
  // phase ≈ 0.20
  // phase ≈ 0.70
  // ------------------------------------------------------


  bool stanceState[4];
  float targetX[4];
  float targetZ[4];
  for(int i=0;i<4;i++)
  {
      Motion_calculateStep(
          i,
          currentPhase,
          targetX[i],
          targetZ[i],
          stanceState[i]
      );
  }

  //=====GAIT_WALK 预判即将抬腿，提前预移重心【新版本】====
  float rawCgX = 0.0f;
  float rawCgY = 0.0f;

  const float CG_SHIFT_MM_X = 8.0f;
  const float CG_SHIFT_MM_Y = 7.0f;
  const float PRE_SHIFT_TIME = 0.12f; // 抬腿之前，提前多久开始挪重心（周期比例）
  const float BDX_Time = Motion_getBDX_Time();

  if(currentGait == GAIT_WALK)
  {
      int swingLeg = -1;
      float timeToSwing = 100.0f; //距离下一次抬腿还有多少周期，初始很大

      // walkStart[4] 每条腿的摆动起始相位
      static const float walkStart[4] =
      {
          0.50f,  //0 右后
          0.25f,  //1 左后
          0.00f,  //2 左前
          0.75f   //3 右前
      };

      //遍历4条腿：计算每条腿距离它自己要抬腿还有多少时间(周期0~1)
      for(int i=0;i<4;i++)
      {
          float s = walkStart[i];
          float delta = s - currentPhase;
          if(delta < 0.0f) delta += 1.0f;

          // 如果这条腿【马上就要抬腿】：距离抬腿 < PRE_SHIFT_TIME
          if(delta >= 0.0f && delta <= PRE_SHIFT_TIME)
          {
              if(delta < timeToSwing)
              {
                  timeToSwing = delta;
                  swingLeg = i;
              }
          }
      }

      if(swingLeg != -1)
      {
          // timeToSwing：距离真正抬腿还剩多少时间
          // 0 → PRE_SHIFT_TIME ：timeToSwing从 PRE_SHIFT_TIME 降到0
          float progress = 1.0f - (timeToSwing / PRE_SHIFT_TIME);
          // progress:0=刚开始预移，1=到达抬腿时刻，重心偏移拉满

          float baseOffsetX = 0.0f;
          float baseOffsetY = 0.0f;
          // 0右后、1左后：后腿抬起 →足端向前(Y>0)，机身向后坐
          // 2左前、3右前：前腿抬起 →足端向后(Y<0)，机身向前倾
          switch(swingLeg)
          {
              case 0: //右后
                  baseOffsetX = -CG_SHIFT_MM_X;
                  baseOffsetY = CG_SHIFT_MM_Y;
                  break;
              case 1: //左后
                  baseOffsetX = -CG_SHIFT_MM_X;
                  baseOffsetY = CG_SHIFT_MM_Y;
                  break;
              case 2: //左前
                  baseOffsetX = CG_SHIFT_MM_X;
                  baseOffsetY = -CG_SHIFT_MM_Y;
                  break;
              case 3: //右前
                  baseOffsetX = CG_SHIFT_MM_X;
                  baseOffsetY = -CG_SHIFT_MM_Y;
                  break;
          }
          rawCgX = baseOffsetX * progress;
          rawCgY = baseOffsetY * progress;
      }
      else
      {
          rawCgX = 0.0f;
          rawCgY = 0.0f;
      }
  }

  //低通滤波
  const float CG_ALPHA = 0.06f;
  filtCgX = filtCgX + CG_ALPHA * (rawCgX - filtCgX);
  filtCgY = filtCgY + CG_ALPHA * (rawCgY - filtCgY);
  //====================================================

  BalanceOffset balance = BalanceController_update(stanceState);

  for (int i = 3; i >= 0; i--) {
      Leg_IK_move(leg[i], targetX[i] + balance.x[i] + filtCgX + filtCgY, targetZ[i] + balance.z[i]);
  }
}



//停止移动
void Motion_stop() {
  Serial.println(">> 停止，回到当前滑杆姿态");
  joystickX = 0.0f;
  joystickY = 0.0f;
  stepLength = 0.0f;
  turnRadius = 0.0f;
  currentPhase = 0.0f;

  isWalking = false;
  isMarching = false;
  lastControlTime = millis();

  Motion_applyStandingPose();
}


//停止移动且复原自己调整的参数
void Motion_resetPose() {//机身复位,包括当前作用在机器人上的   和    控制器上的

  Serial.println("进入Motion_resetPose函数");
  Serial.println(">> 姿态复位");

  joystickX = 0.0f;
  joystickY = 0.0f;
  stepLength = 0.0f;
  turnRadius = 0.0f;
  currentPhase = 0.0f;


  speedScale = 0.75f;
  bodyHeightOffset = 0.0f;
  bodyPitchOffset = 0.0f;
  bodyRollOffset = 0.0f;
  liftScale = 1.0f;
  liftHeight = DEFAULT_LIFT_HEIGHT;
  cycleTime = DEFAULT_CYCLE_TIME;
  


  isWalking = false;
  isMarching = false;
  lastTime = millis();
  lastControlTime = millis();

  Motion_applyStandingPose();
}


//目的是通过串口发送踏步命令,但是和Motion_runAction函数功能好像有点重合,因此待定删除s
void Motion_startMarching(float step, float turn) {
  stepLength = step;
  turnRadius = turn;

  joystickY = constrain(step / DEFAULT_STEP, -1.0f, 1.0f);
  joystickX = constrain(-turn / TURN_SPEED, -1.0f, 1.0f);

  currentPhase = 0.0f;
  lastTime = millis();
  lastControlTime = millis();
  isWalking = true;
  isMarching = (step == 0.0f && turn == 0.0f);
}


// 速度规划器: speedScale: 0~1: 根据机器人能力生成合理步长和周期 不让网页参数直接破坏步态稳定性
static void Motion_updateSpeedProfile(float speedScale,float joystickY)
{
    speedScale = constrain(speedScale,0.0f,1.0f);

    switch(currentGait){
        //trot
        case GAIT_TROT:
        {
            targetStepLength = joystickY * (15.0f + speedScale * 10.0f);
            targetCycleTime = DEFAULT_CYCLE_TIME - speedScale * 0.20f;
            targetCycleTime = constrain(targetCycleTime, 0.9f, 1.2f);
          break;
        }
        //swing
        case GAIT_WALK:
        {
            targetStepLength = joystickY * (3.0f + speedScale * 4.0f);
            targetCycleTime = 2.4f - speedScale * 0.10f;
            targetCycleTime = constrain(targetCycleTime, 2.3f, 2.4f);
          break;
        }
    }
}

//真摇杆触发数据更新
void Motion_setRealtimeControl(float joyX, float joyY, float newSpeedScale, float newBodyHeight, float newBodyPitch, float newBodyRoll, float newLiftScale) {

  joystickX = clampUnit(joyX);
  joystickY = clampUnit(joyY);

  // 前后允许比较灵敏
  joystickY = applyDeadZone(joystickY, 0.12f);
  // 左右死区大一些，防止直走误转向
  joystickX = applyDeadZone(joystickX, 0.30f);

  // 明显前进/后退的时候，小幅左右偏差认为是手指误差
  if (abs(joystickY) > 0.40f && abs(joystickX) < 0.20f)
  {
      joystickX = 0.0f;
  }

  // 转向曲线柔和化
  joystickX = joystickX * abs(joystickX);

  speedScale = constrain(newSpeedScale, 0.0f, 1.0f);           //接收速度倍率变化摇杆参数
  bodyHeightOffset = constrain(newBodyHeight, -30.0f, 15.0f);  //接收高度变化摇杆参数
  bodyPitchOffset = constrain(newBodyPitch, -18.0f, 18.0f);    //接收前后倾变化摇杆参数
  bodyRollOffset = constrain(newBodyRoll, -18.0f, 18.0f);      //接收左右倾变化摇杆参数
  liftScale = constrain(newLiftScale, 0.5f, 1.5f);             //接受步态抬腿倍率变化摇杆参数

  //以上都是在接受参数,但这些参数中,有些还需处理才能直接应用到Motion步态运动控制变量上

  Motion_updateSpeedProfile(speedScale,joystickY);             //速度规划器: 获取targetStepLength 和 targetCycTime
  liftHeight = DEFAULT_LIFT_HEIGHT * liftScale;                //抬腿高度 = 固定高度 * 抬腿倍率 

  


  // 用户约定 x>0 左转，x<0 右转。旧代码里左转常量是负数，所以这里加负号。
  turnRadius = -joystickX * TURN_SPEED * speedScale;           //每次旋转程度  由默认的旋转程度、摇杆的范围、和速度倍率共同构成



  //判断是否在摇杆操控
  //有两套步态逻辑,摇杆和踏步(摇杆优先级高)
  //为了避免步态重叠,因此设置了如果摇杆要控制机器人就取消踏步逻辑(isMarch==false)
  bool joystickActive = (joystickX != 0.0f || joystickY != 0.0f);//如果摇杆动了,就停止踏步
  if (joystickActive) {
    isMarching = false;    //摇杆动了  isMarching == false;
  }



  //以下代码实现了:机器人走与不走的逻辑控制

  /*
  这是个闭环,iswalking取决于用户想不想走,
  当用户控制控制器推杆,这时候数据会被传递到该函数,该函数会判别用户为想走状态,iswalking就被赋值为了用户想走的true
  而这时机器人上一刻状态没走,所以就会进入if(以初始状态开始走)
  走着走着用户松开控制器推杆,这时传到这个函数时,该函数判别机器人不想走,然后机器人上一刻还是想走的,不会进入重置
  而是机器人也会变成这时不想走状态为false
  在下一次,当用户控制控制器推杆,这时候数据会被传递到该函数,该函数会判别用户为想走状态,此时更好因为上一次走的最后iswalking根据用户的意愿也变为了不想走了
  所以此时更好满足上一刻机器人不走
  而重置的条件就是用户想走他上一刻没走,
  但是只要用户现在不想走了那下一刻机器人的上一刻是否想走一定会变成没走对吧
  
  */
  bool shouldWalk = joystickActive || isMarching;//这个变量代表:我现在应该走吗(有两种情况:1.摇杆被控制(人操作),  2.踏步指令  任何一个都该走)

  if (shouldWalk && !isWalking) {//走有两种情况:1.我上一刻已经在走了,那我现在还应该走,说明我们要继续进行步态周期,  2.上一刻没在走,那我现在应该走,说明这是重新开始的步态周期   (这里上一刻停下来了iswalking == false)
    currentPhase = 0.0f;

    lastTime = millis();
  }

  isWalking = shouldWalk; //随着用户意愿改变
  lastControlTime = millis();

  if (!isWalking && robot_state == STAND) {//不走了就回到原位
    /*
    这个也不是“多余”，而是：保证机器人不会“停在中间步态”
    否则可能出现：脚抬一半停住、重心偏移、关节不回中
    */
    Motion_applyStandingPose();
  }
}

// 设置当前步态
bool Motion_setGait(GaitType gait)
{
    if(gait < 0 || gait >= GAIT_COUNT)
    {
        Serial.println(">> 无效步态");
        return false;
    }


    //为了安全：
    //正在运动的时候切换步态，先停止
    if(isWalking)
    {
        Serial.println(">> 切换步态前先停止当前运动");
        Motion_stop();
    }


    currentGait = gait;

    currentPhase = 0.0f;
    lastTime = millis();


    Serial.print(">> 当前步态切换为: ");
    Serial.println(Motion_getGaitName());

    return true;
}

//放回当前步态名称
const char* Motion_getGaitName()
{
    switch(currentGait)
    {
        case GAIT_TROT:
            return "TROT";

        case GAIT_WALK:
            return "WALK";

        default:
            return "UNKNOWN";
    }
}

//获得当前步态状态
GaitType Motion_getGait()
{
    return currentGait;
}


bool Motion_setGaitByName(String gaitName)
{
    gaitName.trim();
    gaitName.toLowerCase();


    if(gaitName == "trot")
    {
        return Motion_setGait(GAIT_TROT);
    }


    if(gaitName == "walk")
    {
        return Motion_setGait(GAIT_WALK);
    }


    Serial.print(">> 未知步态: ");
    Serial.println(gaitName);

    return false;
}

//扩展动作管理器
bool Motion_runAction(String actionName) {
  actionName.trim();

  if (!Motion_Checked_Rob_Stop()) {   //扩展动作前提:机器人不在运动
    Serial.println("机器人还在运行,请先停止运动");
    return false;
  }

  //当前有两个变量来控制步态周期
  //isWalking  isMarching
  //isWalking == true isMarching == false:摇杆正在控制
  if (actionName == "march" || actionName == "踏步") {  //这里的踏步是通过网页直接控制的
    Serial.println(">> 扩展动作：原地踏步");
    stepLength = 0.0f;
    turnRadius = 0.0f;
    currentPhase = 0.0f;
    lastTime = millis();
    lastControlTime = millis();
    isWalking = true;
    isMarching = true;
    return true;
  }

  return false;
}

//是否在运动
bool Motion_isWalking() {
  return isWalking;
}

//判断机器人是否停止步态运动
bool Motion_Checked_Rob_Stop() {
  return (abs(joystickX) < JOYSTICK_DEAD_ZONE && abs(joystickY) < JOYSTICK_DEAD_ZONE && !isWalking);
}


//-------------------------------------------------------------------
static float applyDeadZone(float value, float deadZone)
{
    float a = abs(value);

    if (a <= deadZone)
        return 0.0f;

    // 把 deadZone~1.0 重新映射成 0~1
    float mapped = (a - deadZone) / (1.0f - deadZone);

    return value > 0.0f ? mapped : -mapped;
}

float get_SpeedScale(){
  return speedScale;
}

float get_bodyHeightOffset(){
  return bodyHeightOffset;
}

float get_bodyPitchOffset(){
  return bodyPitchOffset;
}

float get_bodyRollOffset(){
  return bodyRollOffset;
}

float get_liftScale(){
  return liftScale;
}