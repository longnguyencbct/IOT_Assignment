#include "ReadAndSendTask.h"
#include "sensor/sensor.h"
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ...existing code for ThingsBoard and WiFiClient initialization...

void ReadAndSendTask(void *pvParameters) {
  initSensors(); // Initialize sensors here
  while (1) {
    if (millis() - previousDataSend > telemetrySendInterval) {
      previousDataSend = millis();

      SensorData data = readSensorData();
      if (data.valid) {
        Serial.print("Temperature: ");
        Serial.print(data.temperature);
        Serial.print(" °C, Humidity: ");
        Serial.print(data.humidity);
        Serial.println(" %");

        tb.sendTelemetryData("temperature", data.temperature);
        tb.sendTelemetryData("humidity", data.humidity);
      } else {
        Serial.println("Failed to read from sensors!");
      }

      tb.sendAttributeData("rssi", WiFi.RSSI());
      tb.sendAttributeData("channel", WiFi.channel());
      tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
      tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
      tb.sendAttributeData("ssid", WiFi.SSID().c_str());
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
