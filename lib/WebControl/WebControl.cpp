#include "WebControl.h"

#include <WiFi.h>
#include <WebServer.h>

#include "RobotConfig.h"
#include "Motion.h"
#include "WebPage.h"
#include "Robot_state.h"
#include "Command.h"
static WebServer server(80);
static CommandCallback commandCallback = nullptr;//对应Command.cpp里面的Command_execute函数

//判断当前发送的指令有没有名为name的参数没有就按照默认值设置,有就以float类型读取
static float getArgFloat(const char *name, float defaultValue) {
  if (!server.hasArg(name)) {
    return defaultValue;
  }
  return server.arg(name).toFloat();
}

//初次连接192.168.4.1显示控制器
static void WebControl_handleRoot() {
  server.send_P(200, "text/html", MAIN_PAGE);
  //send_p,代表从 Flash（PROGMEM）读取字符串，不占用 RAM 内存，把网页放在Flash，省内存
}

//基础指令接口(直接转发Command_execute)
static void WebControl_handleAction() {
    
    //缺少参数
    if (!server.hasArg("cmd")) {
      server.send(400, "text/plain", "ERROR: missing cmd");
      return;
    }

    String cmd = server.arg("cmd");
    Serial.print("收到网页指令：");
    Serial.println(cmd);

    // 增加回调空指针保护
    if (commandCallback == nullptr) {
      server.send(500, "text/plain", "ERROR: commandCallback not set");
      return;
    }

    commandCallback(cmd);
    server.send(200, "text/plain", "OK");
}

//摇杆操作接口(处理数据,并转发Motion_setRealtimeControl)(stand)
static void WebControl_handleControl() {
      float x = getArgFloat("x", 0.0f);
      float y = getArgFloat("y", 0.0f);
      float speed = getArgFloat("speed", 0.75f);
      float height = getArgFloat("height", 0.0f);
      float pitch = getArgFloat("pitch", 0.0f);
      float roll = getArgFloat("roll", 0.0f);
      float lift = getArgFloat("lift", 1.0f);

      Motion_setRealtimeControl(x, y, speed, height, pitch, roll, lift);

      server.send(200, "application/json", "{\"ok\":true}");

}

//扩展动作接口(stand)
static void WebControl_handleMoveAction() {

    //缺少参数
    if (!server.hasArg("name")) {
      server.send(400, "application/json", "{\"ok\":false,\"msg\":\"missing name\"}");
      return;
    }

    String name = server.arg("name");
    Serial.print("收到动作指令：");
    Serial.println(name);

    if(commandCallback == nullptr){
      server.send(500, "application/json", "{\"ok\":false,\"msg\":\"callback not set\"}");
      return;
    }

    commandCallback(name);

    server.send(200, "application/json", "{\"ok\":true}");

}


//网页控制系统初始化函数
void WebControl_begin(CommandCallback callback) {
  commandCallback = callback;//这个很重要

  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, 6);//开启ESP32的AP热点模式, 带wifi名称和密码 ,wifi信号通道6

  IPAddress ip = WiFi.softAPIP(); //获取AP模式下本机的ip, 一般是固定192.168.4.1

  Serial.print("热点已启动,IP地址:");
  Serial.println(ip);

  server.on("/", HTTP_GET, WebControl_handleRoot);//初始显示页面
  server.on("/action", HTTP_GET, WebControl_handleAction);
  server.on("/control", HTTP_GET, WebControl_handleControl);
  server.on("/moveAction", HTTP_GET, WebControl_handleMoveAction);//静态动作

  server.begin();

  Serial.println("Web前端 控制模块 初始化完成");
}

//侦测控制器是否发指令(在loop不断监听)
void WebControl_handleClient() {
  server.handleClient();
}
