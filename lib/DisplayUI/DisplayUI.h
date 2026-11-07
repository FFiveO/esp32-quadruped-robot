#pragma once

#include <Arduino.h>

//OLED显示模式(IMU数据 / 表情系统)
enum DisplayMode
{
  DISPLAY_NORMAL,
  DISPLAY_FACE
};

// 表情动画类型
typedef enum
{
    //基础表情系统,上电动画 + 待机动画
    FACE_STATE_BOOT,     // 上电开机动画
    FACE_STATE_IDLE,     // 待机常驻，显示COMMON睁眼
    FACE_STATE_BLINK,    // 眨眼动画

    //外部可调用的业务表情ID
    FACE_ID_HELPLESSNESS,    // 茫然
    FACE_ID_EYE_BLIN_BLIN,   // 眼睛闪闪
    FACE_ID_HAPPY_CAT,       // 开心猫咪
    FACE_ID_SEEK,            // 搜索
    
} FaceState;



void DisplayUI_begin();

void OLED_update();

void DisplayUI_setMode();

DisplayMode DisplayUI_getMode();


/**
 * @brief 请求播放指定外部表情
 * @param animId 要播放的表情ID
 * @return true=接受请求，开始播放；false=当前正忙，拒绝请求（正在播放别的外部表情）
 * @note 表情播放完成后自动切回待机眨眼循环；声音逻辑在此处预留，本版本不实现声音，上层业务处理声音
 */
bool DisplayUI_playExternalAnim(FaceState animId);

/**
 * @brief 查询外部表情是否已经播放完成
 * @return true 播放结束；调用后会清除完成标记，系统回到待机
 */
bool DisplayUI_isExternalAnimFinished();