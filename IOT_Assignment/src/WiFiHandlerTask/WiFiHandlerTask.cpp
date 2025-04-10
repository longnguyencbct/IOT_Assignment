#include "WiFiHandlerTask.h"
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

bool reconnect() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }
  InitWiFi();
  return true;
}

void WiFiHandlerTask(void *pvParameters) {
  while (1) {
    if (!reconnect()) {
      vTaskDelay(500 / portTICK_PERIOD_MS);
      continue;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
