#include "ServoSteering.h"

void ServoSteering::setup(const uint8_t gpioPin) {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  this->setPeriodHertz(50); // standard 50 hz servo
  this->attach(gpioPin); // attaches the servo  
}
