#ifndef VOICEMANAGER_H
#define VOICEMANAGER_H

#include <Arduino.h>

enum VoiceSound
{
    SOUND_HELLO,

    SOUND_HAPPY,

    SOUND_CURIOUS,

    SOUND_SLEEPY,

    SOUND_LONG,

    SOUND_ALERT,

    SOUND_BORED_HELPLESSNESS,

    SOUND_BORED_EYE_BLIN_BLIN,

    SOUND_BORED_HAPPY_CAT,

    SOUND_BORED_SEEK
};

void Voice_begin();
void Voice_Button_Control(String voice);
void Voice_Insert_Queue(VoiceSound sound);
bool Voice_isPlaying();
void Voice_stop();

void Voice_setVolume(float volume); 
float Voice_getVolume();
#endif