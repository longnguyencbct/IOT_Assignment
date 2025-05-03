#define LED_PIN 48
#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12
#define A0_PIN GPIO_NUM_1  // Analog pin for LDR
#define FAN_SIG_PIN GPIO_NUM_6 // SIG pin for the fan
#define FAN_NC_PIN GPIO_NUM_7  // NC pin (not connected, can be ignored)


#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "LDR.h"
#include "Wire.h"
#include <ArduinoOTA.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

constexpr char WIFI_SSID[] = ".";//TODO
constexpr char WIFI_PASSWORD[] = "12345679";//TODO

constexpr char TOKEN[] = "qrn77ftmnhsf8vsupkhl";//TODO

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

constexpr char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
constexpr char LED_MODE_ATTR[] = "ledMode";
constexpr char LED_STATE_ATTR[] = "ledState";

volatile bool attributesChanged = false;
volatile int ledMode = 0;
volatile bool ledState = false;

constexpr uint16_t BLINKING_INTERVAL_MS_MIN = 10U;
constexpr uint16_t BLINKING_INTERVAL_MS_MAX = 60000U;
volatile uint16_t blinkingInterval = 1000U;

uint32_t previousStateChange;

constexpr int16_t telemetrySendInterval = 10000U;
uint32_t previousDataSend;

constexpr std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST = {
  LED_STATE_ATTR,
  BLINKING_INTERVAL_ATTR
};

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

DHT20 dht20;

// Initialize LDR sensor
constexpr unsigned long LDR_RESISTOR = 10000; // Replace with your resistor value in ohms
LDR ldr(A0_PIN, LDR_RESISTOR, LDR::GL5528, 10, 0); // Use LDR::GL5528 to specify the enum value

RPC_Response setFanSwitchState(const RPC_Data &data) {
    Serial.println("Received Fan Switch state");
    bool newState = data;
    Serial.print("Fan state change: ");
    Serial.println(newState);
    digitalWrite(FAN_SIG_PIN, newState ? HIGH : LOW); // Control the fan SIG pin
    attributesChanged = true;
    return RPC_Response("setFanSwitchValue", newState);
}

const std::array<RPC_Callback, 1U> callbacks = {
  RPC_Callback{ "setFanSwitchValue", setFanSwitchState }
};

void processSharedAttributes(const Shared_Attribute_Data &data) {
  for (auto it = data.begin(); it != data.end(); ++it) {
    if (strcmp(it->key().c_str(), BLINKING_INTERVAL_ATTR) == 0) {
      const uint16_t new_interval = it->value().as<uint16_t>();
      if (new_interval >= BLINKING_INTERVAL_MS_MIN && new_interval <= BLINKING_INTERVAL_MS_MAX) {
        blinkingInterval = new_interval;
        Serial.print("Blinking interval is set to: ");
        Serial.println(new_interval);
      }
    } else if (strcmp(it->key().c_str(), LED_STATE_ATTR) == 0) {
      ledState = it->value().as<bool>();
      digitalWrite(LED_PIN, ledState);
      Serial.print("LED state is set to: ");
      Serial.println(ledState);
    }
  }
  attributesChanged = true;
}

const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
const Attribute_Request_Callback attribute_shared_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been successfully established
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

const bool reconnect() {
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }
  // If we aren't establish a new connection to the given WiFi network
  InitWiFi();
  return true;
}

void WiFiTask(void *pvParameters) {
  while (1) {
    if (!reconnect()) {
      vTaskDelay(500 / portTICK_PERIOD_MS);
      continue;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void ThingsBoardTask(void *pvParameters) {
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
      tb.sendAttributeData(LED_STATE_ATTR, digitalRead(LED_PIN));
    }

    tb.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void SensorTask(void *pvParameters) {
  while (1) {
    if (millis() - previousDataSend > telemetrySendInterval) {
      previousDataSend = millis();

      dht20.read();
      
      float temperature = dht20.getTemperature();
      float humidity = dht20.getHumidity();
      float lux = ldr.getCurrentLux(); // Get current lux value

      int rawValue = analogRead(A0_PIN);
      Serial.print("Raw Analog Value: ");
      Serial.println(rawValue);

      if (isnan(temperature) || isnan(humidity)) {
        Serial.println("Failed to read from DHT20 sensor!");
      } else {
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print(" °C, Humidity: ");
        Serial.print(humidity);
        Serial.print(" %");

        tb.sendTelemetryData("temperature", temperature);
        tb.sendTelemetryData("humidity", humidity);
      }

      if (!isnan(lux)) {
        Serial.print(", Light Intensity: ");
        Serial.print(lux);
        Serial.println(" lux\n");

        tb.sendTelemetryData("lux", lux); // Send lux telemetry
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

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  pinMode(LED_PIN, OUTPUT);
  pinMode(FAN_SIG_PIN, OUTPUT); // Configure SIG pin as output
  delay(1000);
  InitWiFi();

  Wire.begin(SDA_PIN, SCL_PIN);
  dht20.begin();
  ldr.setPhotocellPositionOnGround(false); // Set photocell to be on the ground

  // Start the fan
  // digitalWrite(FAN_SIG_PIN, HIGH); // Send HIGH signal to start the fan

  xTaskCreate(WiFiTask, "WiFi Task", 4096, NULL, 1, NULL);
  xTaskCreate(ThingsBoardTask, "ThingsBoard Task", 8192, NULL, 1, NULL);
  xTaskCreate(SensorTask, "Sensor Task", 4096, NULL, 1, NULL);
}

void loop() {
  // Empty loop as tasks are managed by FreeRTOS
}