#ifndef SENSOR_H
#define SENSOR_H

#include "DHT20.h"

struct SensorData {
  float temperature;
  float humidity;
  bool valid;
};

extern DHT20 dht20;

void initSensors();
SensorData readSensorData();

#endif // SENSOR_H
