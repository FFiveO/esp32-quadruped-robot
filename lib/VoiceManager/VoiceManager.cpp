#include "VoiceManager.h"
#include <driver/i2s.h>
#include <math.h>
#include "Command.h"
#include <esp_system.h>
#include "Robot_state.h"
// ===================== I2S配置 =====================
#define AUDIO_LRCLK 25
#define AUDIO_BCLK  26
#define AUDIO_DATA  22
#define AUDIO_I2S_PORT I2S_NUM_0
#define AUDIO_SAMPLE_RATE 16000

float MASTER_VOLUME = 0.15f;
float VOICE_GAIN = 8.0f;

// ===================== 后台任务/队列 =====================
static TaskHandle_t voiceTaskHandle = nullptr;
static QueueHandle_t voiceQueue = nullptr;
static volatile bool voicePlaying = false;
static volatile bool stopRequested = false;


// ===================== 音素结构体 & 音素库 =====================
struct Phoneme {
    float baseFreq;     //开始Hz
    float endFreq;      //结束Hz
    int durationMs;     //持续时间
    float volume;       //音量
    float wobbleSpeed;  //抖动
    float wobbleAmount; //幅度
};

// ===================== 全局声音风格 =====================
struct VoiceStyle
{
    //整体音高
    float pitchScale;
    //机器人共振强度
    float formantScale;
    //颤音强度
    float vibratoAmount;
    //机械感噪声
    float noiseAmount;
};

static VoiceStyle voiceStyle = {
    0.6f,   //降低音调
    1.0f,    //默认共振
    0.5f,    //先关闭颤音
    0.05f    //噪声
};


static const Phoneme voiceHello[] = {               //打招呼
    {520, 700, 80, 0.15f, 8, 10},
    {700, 900, 90, 0.18f, 10, 12},
    {900, 650, 120, 0.15f, 6, 8},
};

static const Phoneme voiceHappy[]={                 //开心
    {850, 1500, 90, 0.22f, 25, 90},
    {1100, 1700, 120, 0.20f, 22, 80},
    {1500, 700, 160, 0.16f, 15, 60}
};

static const Phoneme voiceCurious[]={               //好奇
    {500,900,160,0.18f,12,40},
    {700,1400,220,0.20f,18,70},
    {1200,600,180,0.15f,10,35}
};

static const Phoneme voiceSleepy[]={                //困
    {350,320,400,0.12,2,3}
};

static const Phoneme voiceLong[]={                  //长叫
    {600,900,250,0.18f,10,35},
    {900,1500,500,0.22f,18,70},
    {1500,1600,700,0.20f,22,90},
    {1600,700,400,0.16f,12,50}
};


static const Phoneme voiceBoredHelplessness[] = {   //无助
    {520, 430, 900, 0.14f, 0, 0},
    {430, 340, 1100, 0.12f, 0, 0}
};

static const Phoneme voiceBoredEyeBlinBlin[] = {    //眼睛闪烁

    {700, 1050, 650, 0.16f, 0, 0},
    {1050, 1350, 650, 0.17f, 0, 0},
    {1350, 900, 650, 0.14f, 0, 0}
};

static const Phoneme voiceBoredHappyCat[] = {       //开心喵喵叫

    {800, 1300, 600, 0.17f, 0, 0},
    {1300, 1550, 550, 0.18f, 0, 0},
    {1550, 850, 700, 0.15f, 0, 0}
};

static const Phoneme voiceBoredSeek[] = {           //搜索

    {500, 750, 850, 0.14f, 0, 0},
    {750, 1250, 1150, 0.16f, 0, 0}
};


//静音
static void Audio_silence(int durationMs)
{
    int totalSamples = AUDIO_SAMPLE_RATE * durationMs / 1000;
    int16_t data[2] = {0,0};
    for(int i=0; i<totalSamples; i++)
    {
        if(stopRequested) return;
        size_t written;
        i2s_write(AUDIO_I2S_PORT, data, sizeof(data), &written, portMAX_DELAY);
    }
}


//核心发声合成器（带共振峰元音）
static void Audio_petChirp(
    float baseFreq, float endFreq, int durationMs,
    float volume, float wobbleSpeed, float wobbleAmount
)
{
    int totalSamples = AUDIO_SAMPLE_RATE * durationMs / 1000;
    float phase = 0.0f;

    // ========== 双共振峰状态（F1、F2两组IIR，每次音素重置） ==========
    float f1_prev1 = 0.0f, f1_prev2 = 0.0f;
    float f2_prev1 = 0.0f, f2_prev2 = 0.0f;

    // 低频随机漂移替代固定正弦颤音，减少机械感
    float pitchDrift = 0.0f;

    for (int i = 0; i < totalSamples; i++)
    {
        if (stopRequested) return;
        float t = (float)i / AUDIO_SAMPLE_RATE;
        float progress = (float)i / (float)totalSamples;

        // 布朗游走式pitch漂移（替代固定正弦wobble）
        pitchDrift += ((esp_random()/2147483647.0f - 0.5f) * 2.0f - pitchDrift) * 0.08f;

        float frequency = (baseFreq + (endFreq - baseFreq) * progress) * voiceStyle.pitchScale;
        frequency += pitchDrift * wobbleAmount * voiceStyle.vibratoAmount;
        if (frequency < 120.0f) frequency = 120.0f;

        phase += 2.0f * PI * frequency / AUDIO_SAMPLE_RATE;
        if (phase > 2.0f * PI) phase -= 2.0f * PI;

        // ========= MicroDuck 鸭声双共振峰 F1+F2 =========
        float formantF1 = (550 + sinf(t * 2.8f) * 260) * voiceStyle.formantScale;
        float formantF2 = formantF1 * 1.75f + 220;
        const float Q1 = 2.2f;
        const float Q2 = 2.6f;
        const float fs = AUDIO_SAMPLE_RATE;

        // 生成基础波形
        float sineWave = sinf(phase);
        float triangleWave = 2.0f / PI * asinf(sinf(phase));
        float wave = sineWave * 0.62f + triangleWave * 0.38f;

        // 微弱气息白噪声
        float noise = (esp_random()/2147483647.0f - 0.5f) * voiceStyle.noiseAmount;
        wave += noise;
        wave = constrain(wave, -1.0f, 1.0f);

        // ========== 串联IIR滤波器 F1 -> F2 ==========
        // F1滤波
        float w0_1 = 2 * PI * formantF1 / fs;
        float alpha1 = sinf(w0_1) / (2 * Q1);
        float a0_1 = 1 + alpha1;
        float a1_1 = -2 * cosf(w0_1);
        float a2_1 = 1 - alpha1;
        float b0_1 = alpha1;
        float f1_out = (b0_1 * wave - a1_1 * f1_prev1 - a2_1 * f1_prev2) / a0_1;
        f1_prev2 = f1_prev1;
        f1_prev1 = f1_out;

        // F2滤波（F1输出作为F2输入）
        float w0_2 = 2 * PI * formantF2 / fs;
        float alpha2 = sinf(w0_2) / (2 * Q2);
        float a0_2 = 1 + alpha2;
        float a1_2 = -2 * cosf(w0_2);
        float a2_2 = 1 - alpha2;
        float b0_2 = alpha2;
        float f2_out = (b0_2 * f1_out - a1_2 * f2_prev1 - a2_2 * f2_prev2) / a0_2;
        f2_prev2 = f2_prev1;
        f2_prev1 = f2_out;

        wave = f2_out;

        // 指数包络
        float envelope;
        if (progress < 0.06f) {
            envelope = progress / 0.06f;
        } else {
            float p = (progress - 0.06f) / 0.94f;
            envelope = expf(-p * 2.2f);
        }
        envelope = constrain(envelope, 0.0f, 1.0f);

        float output = (int16_t)(wave * 32767.0f * volume * MASTER_VOLUME * VOICE_GAIN * envelope);
        
        output = constrain(output,-32767, 32767);

        int16_t sample = (int16_t)output;

        int16_t stereo[2] = {sample, sample};
        size_t written;
        i2s_write(AUDIO_I2S_PORT, stereo, sizeof(stereo), &written, portMAX_DELAY);
    }
}




// ===================== MicroDuck 风格音素库 =====================

static void Sound_Hello()
{
    for(int i=0;i<3;i++)
    {
        Phoneme p=voiceHello[i];

        Audio_petChirp(
            p.baseFreq,
            p.endFreq,
            p.durationMs,
            p.volume,
            p.wobbleSpeed,
            p.wobbleAmount
        );

        Audio_silence(40);
    }
}

static void Sound_Happy()
{
    for(int i=0;i<3;i++)
    {
        Phoneme p=voiceHappy[i];

        Audio_petChirp(
            p.baseFreq,
            p.endFreq,
            p.durationMs,
            p.volume,
            p.wobbleSpeed,
            p.wobbleAmount
        );

        Audio_silence(40);
    }
}

static void Sound_Curious(){

    for(int i=0;i<3;i++)
    {
        Phoneme p=voiceCurious[i];

        Audio_petChirp(
            p.baseFreq,
            p.endFreq,
            p.durationMs,
            p.volume,
            p.wobbleSpeed,
            p.wobbleAmount
        );

        Audio_silence(40);
    }
}

static void Sound_Sleepy(){

    for(int i=0;i<1;i++)
    {
        Phoneme p=voiceSleepy[i];

        Audio_petChirp(
            p.baseFreq,
            p.endFreq,
            p.durationMs,
            p.volume,
            p.wobbleSpeed,
            p.wobbleAmount
        );

        Audio_silence(40);
    }
}

static void Sound_Long(){

    for(int i=0;i<4;i++)
    {
        Phoneme p=voiceLong[i];

        Audio_petChirp(
            p.baseFreq,
            p.endFreq,
            p.durationMs,
            p.volume,
            p.wobbleSpeed,
            p.wobbleAmount
        );

        Audio_silence(40);
    }
}


static void Sound_PlayBoredVoice(const Phoneme *phonemes, int count)        //播放无聊模式下的声音函数
{
    for(int i = 0; i < count; i++)
    {
        const Phoneme &p = phonemes[i];

        Audio_petChirp(
            p.baseFreq,
            p.endFreq,
            p.durationMs,
            p.volume,
            p.wobbleSpeed,
            p.wobbleAmount
        );

    }
}

static void Sound_BoredHelplessness()
{
    Sound_PlayBoredVoice(
        voiceBoredHelplessness,
        sizeof(voiceBoredHelplessness) / sizeof(voiceBoredHelplessness[0])
    );
}

static void Sound_BoredEyeBlinBlin()
{
    Sound_PlayBoredVoice(
        voiceBoredEyeBlinBlin,
        sizeof(voiceBoredEyeBlinBlin) / sizeof(voiceBoredEyeBlinBlin[0])
    );
}

static void Sound_BoredHappyCat()
{
    Sound_PlayBoredVoice(
        voiceBoredHappyCat,
        sizeof(voiceBoredHappyCat) / sizeof(voiceBoredHappyCat[0])
    );
}

static void Sound_BoredCurious()
{
    Sound_PlayBoredVoice(
        voiceBoredSeek,
        sizeof(voiceBoredSeek) / sizeof(voiceBoredSeek[0])
    );
}



//初始化
static void Audio_begin()
{
    i2s_config_t config = {};
    config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    config.sample_rate = AUDIO_SAMPLE_RATE;
    config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    config.dma_buf_count = 8;
    config.dma_buf_len = 256;
    config.use_apll = false;
    config.tx_desc_auto_clear = true;
    config.fixed_mclk = 0;

    i2s_pin_config_t pins = {};

    //其他都是固定配置
    //----------------------------------
    pins.bck_io_num = AUDIO_BCLK;
    pins.ws_io_num = AUDIO_LRCLK;
    pins.data_out_num = AUDIO_DATA;
    //----------------------------------
    
    pins.data_in_num = I2S_PIN_NO_CHANGE;

    i2s_driver_install(AUDIO_I2S_PORT, &config, 0, nullptr);
    i2s_set_pin(AUDIO_I2S_PORT, &pins);
    i2s_zero_dma_buffer(AUDIO_I2S_PORT);
    Serial.println("Voice I2S initialized");
}


//声音名称查询
static const char* Voice_getSoundName(VoiceSound sound)
{
    switch(sound)
    {
        case SOUND_HELLO:
            return "HELLO";

        case SOUND_HAPPY:
            return "HAPPY";

        case SOUND_CURIOUS:
            return "CURIOUS";

        case SOUND_SLEEPY:
            return "SLEEPY";

        case SOUND_LONG:
            return "LONG";

        case SOUND_ALERT:
            return "ALERT";

        default:
            return "UNKNOWN";
    }
}


static void Voice_executeSound(VoiceSound sound)
{
    switch(sound)
    {

        case SOUND_HELLO:
            Sound_Hello();
            break;

        case SOUND_HAPPY:
            Sound_Happy();
            break;

        case SOUND_CURIOUS:
            Sound_Curious();
            break;

        case SOUND_SLEEPY:
            Sound_Sleepy();
            break;

        case SOUND_LONG:
            Sound_Long();
            break;

        case SOUND_BORED_HELPLESSNESS:
            Sound_BoredHelplessness();
            break;

        case SOUND_BORED_EYE_BLIN_BLIN:
            Sound_BoredEyeBlinBlin();
            break;

        case SOUND_BORED_HAPPY_CAT:
            Sound_BoredHappyCat();
            break;

        case SOUND_BORED_SEEK:
            Sound_BoredCurious();
            break;
    }
}


//音频后台任务 有没有声音订单？没有：睡觉   有：拿出来播放
static void Voice_task(void *parameter)
{
    VoiceSound sound;
    while (true)
    {
        if(xQueueReceive(voiceQueue, &sound, portMAX_DELAY) == pdTRUE)
        {
            stopRequested = false;
            voicePlaying = true;
            Voice_executeSound(sound);
            i2s_zero_dma_buffer(AUDIO_I2S_PORT);
            voicePlaying = false;
        }
    }
}


//对外API
void Voice_begin()
{
    Audio_begin();
    voiceQueue = xQueueCreate(5, sizeof(VoiceSound));//声音排队队列,假如有三个声音同时来,不能同时播放
    if (!voiceQueue) {  //容错判断：如果内存不够，队列创建失败，打印提示，直接退出初始化，声音系统不启动。
        Serial.println("Voice queue create failed"); 
        return; 
    }
    xTaskCreatePinnedToCore(Voice_task, "VoiceTask", 8192, nullptr, 1, &voiceTaskHandle, 0);
    Serial.println("VoiceManager initialized");
}


//前端按钮发声指令接收端口
void Voice_Button_Control(String voice){
        if(voice=="hello"){
            Voice_Insert_Queue(SOUND_HELLO);
        }
        else if(voice=="happy"){
            Voice_Insert_Queue(SOUND_HAPPY);
        }
        else if(voice=="curious"){
            Voice_Insert_Queue(SOUND_CURIOUS);
        }
        else if(voice=="sleepy"){
            Voice_Insert_Queue(SOUND_SLEEPY);
        }
        else if(voice=="long"){
            Voice_Insert_Queue(SOUND_LONG);
        } 
}

//给后台声音系统发送一个请求
void Voice_Insert_Queue(VoiceSound sound)
{
    if(voiceQueue==nullptr)
        return;

    xQueueSend(voiceQueue, &sound, 0);
}

bool Voice_isPlaying() 
{ 
    return voicePlaying;
}

//停止播放声音
void Voice_stop()
{
    stopRequested = true;
    if(voiceQueue) xQueueReset(voiceQueue);
    i2s_zero_dma_buffer(AUDIO_I2S_PORT);
}

//设置音量
void Voice_setVolume(float volume)
{

    MASTER_VOLUME =constrain(volume, 0.0f, 1.0f);//限制音量

    Serial.print("Voice volume:");
    Serial.println(MASTER_VOLUME);
}

float Voice_getVolume()
{
    return MASTER_VOLUME;
}