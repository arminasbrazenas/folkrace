#include "SharpIRSensor.h"

void SharpIRSensor::setPin(const uint8_t pin)
{
  this->pin = pin;
}

void SharpIRSensor::setFunctionCoeffs(const float M, const float P)
{
  powerCoeffM = M;
  powerCoeffP = P;
}

// Returns distance in millimeters
uint16_t SharpIRSensor::getDistance()
{
  uint16_t analogValue = analogRead(pin);

  return (powerCoeffM * pow(analogValue, powerCoeffP));
}
