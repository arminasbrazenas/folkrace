#ifndef MOTORS_SIDE_H
#define MOTORS_SIDE_H

#include <Arduino.h>

class MotorsSide
{
  public:
    void setPins(uint8_t fwdPin, uint8_t bwdPin);
    void drive(int16_t motorSpeed);
  private:
    uint8_t fwdPin;
    uint8_t bwdPin;
};

#endif
