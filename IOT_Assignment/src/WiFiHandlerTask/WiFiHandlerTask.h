#ifndef WIFIHANDLERTASK_H
#define WIFIHANDLERTASK_H

#include "main.h"

void InitWiFi();
bool reconnect();
void WiFiHandlerTask(void *pvParameters);

#endif // WIFIHANDLERTASK_H
