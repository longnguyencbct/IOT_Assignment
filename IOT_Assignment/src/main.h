#include <Arduino.h> // or #include <stdint.h>

#include <Wire.h>
#include <ArduinoOTA.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "WiFiHandlerTask/WiFiHandlerTask.h"
#include "ServerConnectionHandlerTask/ServerConnectionHandlerTask.h"
#include "ReadAndSendTask/ReadAndSendTask.h"
#include "RemoteTriggerHandlerTask/RemoteTriggerHandlerTask.h"

#ifndef MAIN_H
#define MAIN_H

#define LED_PIN 48

extern const char WIFI_SSID[];
extern const char WIFI_PASSWORD[];

extern const char TOKEN[];
extern const char THINGSBOARD_SERVER[];
extern const uint16_t THINGSBOARD_PORT; // Used by [THINGSBOARD_PORT](c:\D\University\IOT\Assignment\project_workspace\IOT_Assignment\src\main.h)
extern const uint32_t MAX_MESSAGE_SIZE; // Used by [MAX_MESSAGE_SIZE](c:\D\University\IOT\Assignment\project_workspace\IOT_Assignment\src\main.h)
extern const uint32_t SERIAL_DEBUG_BAUD; // Used by [SERIAL_DEBUG_BAUD](c:\D\University\IOT\Assignment\project_workspace\IOT_Assignment\src\main.h)

extern const char BLINKING_INTERVAL_ATTR[];
extern const char LED_MODE_ATTR[];
extern const char LED_STATE_ATTR[];

extern uint32_t previousDataSend;
extern const int16_t telemetrySendInterval;
extern volatile bool attributesChanged;
extern const Shared_Attribute_Callback attributes_callback;
extern const Attribute_Request_Callback attribute_shared_request_callback;
extern std::array<RPC_Callback, 1U> callbacks;

// Declare ThingsBoard object without specifying a buffer size
extern ThingsBoard tb;

#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>

extern WiFiClient wifiClient;
extern Arduino_MQTT_Client mqttClient;

#endif // MAIN_H