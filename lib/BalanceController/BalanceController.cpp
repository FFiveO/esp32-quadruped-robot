#include "BalanceController.h"
#include "IMU.h"
#include "RobotConfig.h"


static float rollKp = 0.45f;
static float pitchKp = 0.45f;

static float rollKd = 0.14f;
static float pitchKd = 0.14f;


// 两套参数
// 普通档位
const float NORM_KP = 0.45f;
const float NORM_KD = 0.14f;
// 超强平衡档位【演示用，增益拉高，注意容易震荡！你可以微调】
const float STRONG_KP = 0.45f;
const float STRONG_KD = 0.14f;

static BalanceModeLevel balanceLevel = BALANCE_NORMAL;


static float lastPitchComp = 0;             //上一次pitch变化后记录的偏移值
static float lastRollComp = 0;

static bool balanceEnabled = false;

static float targetRoll = 0.0f;
static float targetPitch = 0.0f;

static float lastPitchOut = 0.0f;
static float lastRollOut = 0.0f;
// 超强模式每帧最大变化毫米；普通模式可以放宽
const float NORM_STEP_LIMIT = 2.0f;
const float STRONG_STEP_LIMIT = 5.0f;

// 普通模式最大补偿mm
const float NORM_MAX_COMP = 4.0f;
// 超强模式最大补偿mm
const float STRONG_MAX_COMP = 20.0f;


//返回当前平衡姿态
bool BalanceMode_get(){
    return balanceEnabled;
}


// 新增设置档位函数
void BalanceController_setLevel(BalanceModeLevel lv)
{
    balanceLevel = lv;
    if(balanceLevel == BALANCE_NORMAL){
        rollKp = NORM_KP;
        pitchKp = NORM_KP;
        rollKd = NORM_KD;
        pitchKd = NORM_KD;
        Serial.println("平衡档位：普通模式");
    }else if(balanceLevel == BALANCE_STRONG){
        rollKp = STRONG_KP;
        pitchKp = STRONG_KP;
        rollKd = STRONG_KD;
        pitchKd = STRONG_KD;
        Serial.println("平衡档位：超强平衡【演示，增益高，注意震荡】");
            // 如果平衡已经打开，切换到超强模式重新捕获零点！
        if(balanceEnabled)
        {
            targetRoll = IMU_getRoll();
            targetPitch = IMU_getPitch();
            Serial.println(">>> 切换超强平衡，重新采集机身零点");
        }
    
    }
}

//平衡模式"开关"
void BalanceController_setEnabled()
{

    //
    if(!balanceEnabled)//第一次开启平衡模式,记录当前roll pitch
    {
        balanceEnabled = true;

        targetRoll = IMU_getRoll();
        targetPitch = IMU_getPitch();

        Serial.print("目标Roll=");
        Serial.print(targetRoll);
        Serial.print(",   目标Pitch=");
        Serial.println(targetPitch);
        
        Serial.println(">> 平衡补偿已开启");
    }else{
        balanceEnabled = false;
        Serial.println(">> 平衡补偿已关闭");
    }



}


//返回四条腿的补偿 "根据IMU" "根据是否为支撑腿"
BalanceOffset BalanceController_update(bool stance[4])
{
    BalanceOffset offset;
    for(int i=0;i<4;i++){
        offset.x[i]=0;
        offset.z[i]=0;
    }

    if(!balanceEnabled){
        return offset;
    }

    float roll = IMU_getRoll();
    float pitch = IMU_getPitch();

    float rollError = targetRoll - roll;
    float pitchError = targetPitch - pitch;

    float gyroX = IMU_getGyroXRate();
    float gyroY = IMU_getGyroYRate();

    // PD计算
    float rollComp = rollError * rollKp - gyroX * rollKd;
    float pitchComp = pitchError * pitchKp - gyroY * pitchKd;

    // 一阶输出平滑滤波
    pitchComp = lastPitchComp * 0.4f + pitchComp * 0.6f;
    rollComp = lastRollComp * 0.4f + rollComp * 0.6f;

    // =========【第一步 先确定档位对应的两个限制参数】=========
    float maxComp;
    float stepLimit;
    if(balanceLevel == BALANCE_STRONG){
        maxComp = STRONG_MAX_COMP;
        stepLimit = STRONG_STEP_LIMIT;
    }else{
        maxComp = NORM_MAX_COMP;
        stepLimit = NORM_STEP_LIMIT;
    }

    // =========【1.全局最大输出限幅】=========
    pitchComp = constrain(pitchComp, -maxComp, maxComp);
    rollComp  = constrain(rollComp, -maxComp, maxComp);

    // =========【2.单帧步长变化限制，抑制跳变】=========
    pitchComp = constrain(pitchComp, lastPitchOut-stepLimit, lastPitchOut+stepLimit);
    rollComp  = constrain(rollComp,  lastRollOut-stepLimit,  lastRollOut+stepLimit);

    // 保存本次输出，作为下一帧步长限制基准
    lastPitchOut = pitchComp;
    lastRollOut  = rollComp;

    // 更新滤波历史缓存
    lastPitchComp = pitchComp;
    lastRollComp  = rollComp;

    // 支撑/摆动腿权重
    float weight[4];
    for(int i=0;i<4;i++){
        weight[i] = stance[i] ? 1.0f : 0.0f;
    }

    // Z轴补偿分配
    offset.z[0] += (pitchComp+rollComp) * weight[0];//右后
    offset.z[1] += (-pitchComp+rollComp) * weight[1];//左后
    offset.z[2] += (-pitchComp-rollComp) * weight[2];//左前
    offset.z[3] += (pitchComp-rollComp) * weight[3];//右前

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200)
    {
        lastPrint = millis();
        Serial.print("pitch=");
        Serial.print(pitch);
        Serial.print("  gx=");
        Serial.print(IMU_getGyroXRate());
        Serial.print("  gy=");
        Serial.println(IMU_getGyroYRate());
    }
    return offset;
}