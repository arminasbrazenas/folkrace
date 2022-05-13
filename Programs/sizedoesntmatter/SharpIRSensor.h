#ifndef SHARP_IR_SENSOR_H
#define SHARP_IR_SENSOR_H

#include <Arduino.h>

class SharpIRSensor
{
  public:
    void setPin(const uint8_t pin);
    void setFunctionCoeffs(const float M, const float P);
    uint16_t getDistance();
  private:
    uint8_t pin;
    float powerCoeffM;
    float powerCoeffP;
};

#endif
