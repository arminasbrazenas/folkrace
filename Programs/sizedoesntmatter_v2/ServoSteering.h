#ifndef SERVO_STEERING_H
#define SERVO_STEERING_H

#include <ESP32Servo.h>

class ServoSteering : public Servo {
  public:
    void setup(const uint8_t gpioPin);
};

#endif SERVO_STEERING_H
