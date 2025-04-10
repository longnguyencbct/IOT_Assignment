#include "ServerConnectionHandlerTask.h"
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ...existing code for ThingsBoard and WiFiClient initialization...

void ServerConnectionHandlerTask(void *pvParameters) {
  while (1) {
    if (!tb.connected()) {
      Serial.print("Connecting to: ");
      Serial.print(THINGSBOARD_SERVER);
      Serial.print(" with token ");
      Serial.println(TOKEN);
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
        Serial.println("Failed to connect");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        continue;
      }

      tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());

      Serial.println("Subscribing for RPC...");
      if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
        Serial.println("Failed to subscribe for RPC");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        continue;
      }

      if (!tb.Shared_Attributes_Subscribe(attributes_callback)) {
        Serial.println("Failed to subscribe for shared attribute updates");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        continue;
      }

      Serial.println("Subscribe done");

      if (!tb.Shared_Attributes_Request(attribute_shared_request_callback)) {
        Serial.println("Failed to request for shared attributes");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        continue;
      }
    }

    if (attributesChanged) {
      attributesChanged = false;
      tb.sendAttributeData("ledState", digitalRead(LED_PIN));
    }

    tb.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
