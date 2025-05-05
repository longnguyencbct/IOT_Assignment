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
#include <TensorFlowLite_ESP32.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
// #include <HTTPClient.h>
#include <ArduinoHttpClient.h>

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

extern unsigned char my_model_tflite[];
extern unsigned int my_model_tflite_len;

constexpr int kTensorArenaSize = 16 * 1024;
static uint8_t tensor_arena[kTensorArenaSize];

const int n_steps = 10;           // for example
const int n_features = 2;         // humidity, temperature
float humi_seq[n_steps];
float temp_seq[n_steps];
int seq_index = 0;
bool seq_full = false;

namespace {
  tflite::MicroInterpreter *interpreter;
  const tflite::Model *model;  
  TfLiteTensor *input;
  TfLiteTensor *output;
}

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
  WiFiClient wifiClient;
  HttpClient http(wifiClient, "192.168.43.117", 5000); // Replace with the actual IP and port of the REST API
  bool isHttpClientInitialized = false;

  while (1) {
    if (millis() - previousDataSend > telemetrySendInterval) {
      previousDataSend = millis();

      dht20.read();
      
      float temperature = dht20.getTemperature();
      float humidity = dht20.getHumidity();
      float lux = ldr.getCurrentLux(); // Get current lux value

      humi_seq[seq_index] = humidity;
      temp_seq[seq_index] = temperature;
      seq_index += 1;
      if(seq_index > n_steps) {
          seq_index = 0;
          seq_full = true;
      }
      if(seq_full) {
        for(int i = 0; i < n_steps; ++i) {
          interpreter->input(0)->data.f[i * 2] = humi_seq[i];
          interpreter->input(0)->data.f[i * 2 + 1] = temp_seq[i];
        }

        if(interpreter->Invoke() != kTfLiteOk) {
          Serial.println("Invoke failed!");
          return;
        }
        float predicted_humidity = interpreter->output(0)->data.f[0];
        float predicted_temperature = interpreter->output(0)->data.f[1];

        Serial.print("Predicted Humidity: ");
        Serial.println(predicted_humidity);
        Serial.print("Predicted Temperature: ");
        Serial.println(predicted_temperature);
      }

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

      // Initialize HTTP connection only once
      if (!isHttpClientInitialized) {
        Serial.println("Initializing HTTP connection...");
        isHttpClientInitialized = true;
      }

      // Send data to the REST API using ArduinoHttpClient
      http.beginRequest();
      http.post("/api/telemetry");
      http.sendHeader("Content-Type", "application/json");

      String payload = "{";
      payload += "\"datetime\":\"" + String(millis()) + "\",";
      payload += "\"temperature\":" + String(temperature) + ",";
      payload += "\"humidity\":" + String(humidity) + ",";
      payload += "\"lux\":" + String(lux);
      payload += "}";

      http.sendHeader("Content-Length", payload.length());
      http.print(payload);

      int httpResponseCode = http.responseStatusCode();
      String response = http.responseBody();

      if (httpResponseCode > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        Serial.print("Response: ");
        Serial.println(response);
      } else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
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

  model = tflite::GetModel(my_model_tflite);
  static tflite::AllOpsResolver resolver;

  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize, nullptr);

  interpreter = &static_interpreter;

  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if(allocate_status != kTfLiteOk) {
    Serial.println("AllocateTensors() failed");
    while (1);
  }

  input = interpreter->input(0);
  output = interpreter->output(0);

  xTaskCreate(WiFiTask, "WiFi Task", 4096, NULL, 1, NULL);
  xTaskCreate(ThingsBoardTask, "ThingsBoard Task", 8192, NULL, 1, NULL);
  xTaskCreate(SensorTask, "Sensor Task", 4096, NULL, 1, NULL);
}

void loop() {
  // Empty loop as tasks are managed by FreeRTOS
}
 