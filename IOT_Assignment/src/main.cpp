#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12

#include "main.h"

const char WIFI_SSID[] = "."; // TODO
const char WIFI_PASSWORD[] = "12345679"; // TODO

const char TOKEN[] = "qrn77ftmnhsf8vsupkhl"; // TODO

const char THINGSBOARD_SERVER[] = "app.coreiot.io";
const uint16_t THINGSBOARD_PORT = 1883U;

const uint32_t MAX_MESSAGE_SIZE = 1024U;
const uint32_t SERIAL_DEBUG_BAUD = 115200U;

const char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
const char LED_MODE_ATTR[] = "ledMode";
const char LED_STATE_ATTR[] = "ledState";

uint32_t previousDataSend = 0; // Tracks the last time telemetry data was sent
const int16_t telemetrySendInterval = 3000; // Interval in milliseconds for sending telemetry data


ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);
volatile bool attributesChanged = false; // Tracks if attributes have changed
const Shared_Attribute_Callback attributes_callback = Shared_Attribute_Callback(processSharedAttributes);
const Attribute_Request_Callback attribute_shared_request_callback = Attribute_Request_Callback(processSharedAttributes);
std::array<RPC_Callback, 1U> callbacks = {RPC_Callback("setLedSwitchState", setLedSwitchState)};

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  pinMode(LED_PIN, OUTPUT);
  delay(1000);

  xTaskCreate(WiFiHandlerTask, "WiFi Handler Task", 4096, NULL, 1, NULL);
  xTaskCreate(ServerConnectionHandlerTask, "Server Connection Handler Task", 8192, NULL, 1, NULL);
  xTaskCreate(ReadAndSendTask, "Read and Send Task", 4096, NULL, 1, NULL);
  xTaskCreate(RemoteTriggerHandlerTask, "Remote Trigger Handler Task", 4096, NULL, 1, NULL);
}

void loop() {
  // Empty loop as tasks are managed by FreeRTOS
}