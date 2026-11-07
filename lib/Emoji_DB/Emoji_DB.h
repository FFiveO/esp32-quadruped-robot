// Emojis.h
#pragma once
#include <Arduino.h>

// 定义帧数，方便修改
#define WAKE_FRAME_SIZE 34//开机表情数组帧数

#define HELPLESSNESS_FRAME_SIZE 19//茫然表情数组帧数

#define EYE_BLIN_BLIN_FRAME_SIZE 20//眼睛闪闪表情数组帧数

#define HAPPY_CAT_FRAME_SIZE 15//开心猫咪叫表情数组帧数 

#define BLINK_FRAME_SIZE 6//眨眼表情数组帧数

#define SEEK_FRAME_SIZE 20//好奇表情数组帧数


// 声明外部数组
extern const unsigned char* OLED_WAKE[WAKE_FRAME_SIZE];//开机表情数组

extern const unsigned char* OLED_HELPLESSNESS[HELPLESSNESS_FRAME_SIZE];//茫然表情数组

extern const unsigned char* OLED_EYE_BLIN_BLIN[EYE_BLIN_BLIN_FRAME_SIZE];//眼睛闪闪数组

extern const unsigned char* OLED_HAPPY_CAT[HAPPY_CAT_FRAME_SIZE];//开心猫咪叫数组

extern const unsigned char* OLED_BLINK[BLINK_FRAME_SIZE];//眨眼数组

extern const unsigned char* OLED_SEEK[SEEK_FRAME_SIZE];//搜索数组

extern const unsigned char COMMON [] PROGMEM;