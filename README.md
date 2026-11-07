# esp32-quadruped-robot
ESP32 四足机器狗 | PlatformIO 项目
基于ESP32开发的小型四足机器人，实现姿态平衡、逆运动学步态规划与无线网页控制。

## ✨ 功能 Features
- ✅ 逆运动学IK解算，四足足端轨迹规划
- ✅ PD姿态平衡控制器，抗倾斜自稳
- ✅ 多步态逻辑：站立、趴下、原地踏步、行走
- ✅ WiFi网页上位机：无线调参、实时控制
- ✅ I2S音频合成，动作完成提示音效
- ✅ MPU6050姿态采集，PCA9685舵机驱动
- ✅ 状态机管理机器人行为（无聊自动互动）

## 🛠️ 硬件清单 Hardware
- 主控：ESP32
- 舵机驱动：PCA9685
- 姿态传感器：MPU6050
- 音频模块：I2S DAC
- 舵机：8 × SG90

## 🧰 开发环境
- VSCode + PlatformIO
- 框架：ESP32 Arduino

## 📁 项目目录
├── include/ 公共头文件
├── lib/ 自定义模块库（平衡、IK、语音、网页控制等）
├── src/ main 主程序入口
├── test/ 测试代码
├── platformio.ini PlatformIO 编译配置

## 📹 演示视频
> 后续把B站演示链接贴在这里

## 📄 License
MIT
