#ifndef DISTANCE_SENSORS_H
#define DISTANCE_SENSORS_H

#include "Arduino.h"

class DistanceSensor {
  private:
    uint8_t pin;

  public:
    void setPin(const uint8_t _pin);
    void setup();
    int32_t getDistance(); // In millimeters
};

class DistanceSensors {
  private:
    uint8_t total;

  public:
    DistanceSensor sensors[7];
    DistanceSensors(uint8_t _total, uint8_t pins[]);
    int32_t getDifference();
    void setup();
};

#endif
