#include "DisplayUI.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RobotConfig.h"
#include "IMU.h" //因为屏幕需要显示imu的信息
#include "Motion.h"
#include "Emoji_DB.h"
#include "BoredSystem.h"
#include "BalanceController.h"

static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);//这里的display是程序运行期间的屏幕管理员


static DisplayMode displayMode = DISPLAY_FACE;  //默认OLED显示模式是表情系统
static FaceState faceState = FACE_STATE_BOOT;

// 动画控制变量
static uint16_t curFrameIdx = 0;
static unsigned long lastOledFaceTime = 0;
const uint16_t ANIM_FRAME_INTERVAL = 200;   // 动画帧间隔ms


// 外部表情动画控制

static const unsigned char** externalAnimFrames = nullptr;     // 当前正在播放的外部动画帧数组
static uint16_t externalAnimFrameCount = 0;     // 当前外部动画总帧数
static bool externalAnimPlaying = false;        // 当前是否正在播放外部业务表情
static bool externalAnimFinished = false;       // 外部动画是否刚刚播放完成


// 待机多久触发一次眨眼
static unsigned long idleStartMs = 0;           //待机多久以后眨眼
const unsigned long IDLE_BLINK_INTERVAL = 3000;


static unsigned long lastOledDataTime = 0;          //记录上次oled刷新IMU数据的时间

// 进入待机状态
static void enterIdleState()
{
    faceState = FACE_STATE_IDLE;

    curFrameIdx = 0;

    idleStartMs = millis();
    lastOledFaceTime = millis();

    // 立即恢复普通睁眼表情
    display.clearDisplay();
    display.drawBitmap(0, 0, COMMON, 128, 64, WHITE);
    display.display();

    Serial.println("进入待机状态");
}



//初始化OLED
void DisplayUI_begin() {
  //Wire.setClock(100000UL); // I2C降到100kHz，稳定性显著提升
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {//初始化失败
    Serial.println("OLED 初始化失败");
    return;
  }

  display.clearDisplay();//把内存里的“画板”擦得干干净净（但此时物理屏幕上可能还有残留画面）
  display.display();//把内存里最新的内容，一次性“推”到物理屏幕上显示出来

  //上面相当于给屏幕做了一次彻底的“黑屏复位”
  faceState = FACE_STATE_BOOT;
  curFrameIdx = 0;            //初始显示表情,默认帧数从0开始
  lastOledFaceTime = millis();//记录上次显示表情的系统时间为"现在"    

  lastOledDataTime = millis();//记录上次显示IMU数据的系统时间为"现在"

  Serial.println("显示屏 OLED 初始化完成, 开始执行开机表情动画");
}


//更改OLED显示模式
void DisplayUI_setMode(){

  if(displayMode == DISPLAY_NORMAL){
    displayMode = DISPLAY_FACE;

    faceState = FACE_STATE_BOOT;
    curFrameIdx = 0;
    lastOledFaceTime = millis();
    idleStartMs = millis();
    Serial.println("切换至表情模式，重新执行开机动画");

  }else{
    displayMode = DISPLAY_NORMAL;
  }

  display.clearDisplay();
  display.display();
}


//获取OLED显示模式
DisplayMode DisplayUI_getMode(){
    return displayMode;
}


//表情状态机核心
static void updateFace() {
    unsigned long now = millis();
    const unsigned char* framePtr = nullptr;

    switch (faceState)
        {
            // ---------------------- 开机唤醒动画 ----------------------
            case FACE_STATE_BOOT:
            {
                if(now - lastOledFaceTime >= ANIM_FRAME_INTERVAL)
                {
                    lastOledFaceTime = now;
                    // 从PROGMEM读取位图指针
                    framePtr = (const unsigned char*)pgm_read_ptr(&OLED_WAKE[curFrameIdx]);

                    display.clearDisplay();
                    display.drawBitmap(0, 0, framePtr, 128, 64, WHITE);
                    display.display();

                    curFrameIdx ++;
                    // 判断开机动画播放完毕
                    if(curFrameIdx >= WAKE_FRAME_SIZE)
                    {
                        Serial.println("开机动画播放完毕，进入待机状态");
                        enterIdleState();

                        BoredSystem_notifyExpressionFinished();
                        
                    }
                }
                break;
            }

            // ---------------------- 待机状态：显示COMMON睁眼，计时等待眨眼 ----------------------
            case FACE_STATE_IDLE:
            {
                // 计时到触发眨眼动画
                if(now - idleStartMs >= IDLE_BLINK_INTERVAL)
                {
                    Serial.println("待机计时到，播放眨眼");
                    faceState = FACE_STATE_BLINK;
                    curFrameIdx = 0;
                    lastOledFaceTime = millis();
                }
                break;
            }

            // ---------------------- 眨眼动画播放 ----------------------
            case FACE_STATE_BLINK:
            {
                if(now - lastOledFaceTime >= ANIM_FRAME_INTERVAL)
                {
                    lastOledFaceTime = now;
                    framePtr = (const unsigned char*)pgm_read_ptr(&OLED_BLINK[curFrameIdx]);

                    display.clearDisplay();
                    display.drawBitmap(0,0, framePtr, 128,64, WHITE);
                    display.display();

                    curFrameIdx ++;
                    // 眨眼全部帧播放完成，切回待机
                    if(curFrameIdx >= BLINK_FRAME_SIZE)
                    {
                        Serial.println("眨眼动画结束，回到待机");
                        enterIdleState();
                    }
                }
                break;
            }
            // ---------------------- 外部业务表情 ----------------------
            case FACE_ID_HELPLESSNESS:
            case FACE_ID_EYE_BLIN_BLIN:
            case FACE_ID_HAPPY_CAT:
            case FACE_ID_SEEK:
            {
                // 理论上不会发生，只是保护一下
                if(!externalAnimPlaying ||
                  externalAnimFrames == nullptr ||
                  externalAnimFrameCount == 0)
                {
                    externalAnimPlaying = false;

                    enterIdleState();

                    break;
                }


                if(now - lastOledFaceTime >= ANIM_FRAME_INTERVAL)
                {
                    lastOledFaceTime = now;
                    // 读取当前动画帧
                    framePtr = (const unsigned char*)pgm_read_ptr(&externalAnimFrames[curFrameIdx]);

                    display.clearDisplay();

                    display.drawBitmap(0,0,framePtr,128,64,WHITE);

                    display.display();

                    curFrameIdx++;


                    // 当前业务表情播放结束
                    if(curFrameIdx >= externalAnimFrameCount)
                    {
                        Serial.println("外部表情播放结束，恢复待机");
                        externalAnimPlaying = false;
                        externalAnimFinished = true;
                        externalAnimFrames = nullptr;
                        externalAnimFrameCount = 0;

                        // 自动恢复COMMON
                        enterIdleState();
                        //告诉无聊系统：
                        //本次随机表情已经真正播放结束
                        //从现在才开始计算下一次无聊时间
                        BoredSystem_notifyExpressionFinished();
                    }
                }

                break;
            }
        }
}

// 请求播放外部业务表情
//============================================================
bool DisplayUI_playExternalAnim(FaceState animId)
{
    // 当前不是表情模式，不允许播放
    if(displayMode != DISPLAY_FACE)
    {
        return false;
    }

    // 外部表情正在播放，不接受新的请求
    if(externalAnimPlaying)
    {
        return false;
    }

    // 只允许在待机状态接受外部表情
    //
    // 如果现在正在开机动画或者眨眼，
    // 就先让当前动画播放完。
    if(faceState != FACE_STATE_IDLE)
    {
        return false;
    }


    // 根据请求的表情ID选择对应动画资源
    switch(animId)
    {
        case FACE_ID_HELPLESSNESS:
        {
            externalAnimFrames = OLED_HELPLESSNESS;
            externalAnimFrameCount = HELPLESSNESS_FRAME_SIZE;
            break;
        }

        case FACE_ID_EYE_BLIN_BLIN:
        {
            externalAnimFrames = OLED_EYE_BLIN_BLIN;
            externalAnimFrameCount = EYE_BLIN_BLIN_FRAME_SIZE;
            break;
        }

        case FACE_ID_HAPPY_CAT:
        {
            externalAnimFrames = OLED_HAPPY_CAT;
            externalAnimFrameCount = HAPPY_CAT_FRAME_SIZE;
            break;
        }

        case FACE_ID_SEEK:
        {
            externalAnimFrames = OLED_SEEK;
            externalAnimFrameCount = SEEK_FRAME_SIZE;
            break;
        }

        default:
        {
            // 不是合法的业务表情
            return false;
        }
    }


    // 正式开始播放
    faceState = animId;

    curFrameIdx = 0;

    lastOledFaceTime = millis();

    externalAnimPlaying = true;

    externalAnimFinished = false;

    Serial.print("开始播放外部表情, ID = ");
    Serial.println((int)animId);

    return true;
}

//实时显示IMU数据
static void updateImuData(){
  if (millis() - lastOledDataTime < OLED_INTERVAL_MS) {
    return;
  }

  lastOledDataTime = millis();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("RobotDog");

  display.setTextSize(2);
  display.setCursor(0, 14);
  display.print("R:");
  display.println(IMU_getRoll(), 1);

  display.setCursor(0, 38);
  display.print("P:");
  display.println(IMU_getPitch(), 1);

  display.setTextSize(1);
  display.setCursor(88, 0);
  display.print(Motion_isWalking() ? "WALK" : "STOP");

  display.setCursor(88, 10);
  display.print(BalanceMode_get() ? "BAL" : "NO B");

  display.display();
}

//OLED总入口, main loop调用
void OLED_update() {

  if(displayMode == DISPLAY_FACE){//更新表情
      updateFace();   
  }else{
      updateImuData();
  }
  return;
}


// 查询外部表情是否播放完成
bool DisplayUI_isExternalAnimFinished()
{
    if(externalAnimFinished)
    {
        externalAnimFinished = false;

        return true;
    }

    return false;
}

