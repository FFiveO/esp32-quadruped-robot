#pragma once

#include <Arduino.h>

typedef void (*CommandCallback)(String cmd);

void WebControl_begin(CommandCallback callback);

void WebControl_handleClient();