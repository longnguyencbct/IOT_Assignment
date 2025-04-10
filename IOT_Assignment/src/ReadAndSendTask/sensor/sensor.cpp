#include "sensor.h"
#include "Wire.h"

DHT20 dht20;

void initSensors() {
  Wire.begin(GPIO_NUM_11, GPIO_NUM_12); // SDA_PIN, SCL_PIN
  dht20.begin();
}

SensorData readSensorData() {
  SensorData data;
  dht20.read();

  data.temperature = dht20.getTemperature();
  data.humidity = dht20.getHumidity();
  data.valid = !isnan(data.temperature) && !isnan(data.humidity);

  return data;
}
