#ifndef MOTORS_H
#define MOTORS_H

#include "Arduino.h"
#include "Constants.h"

class Motor {
  private:
    class Pins {
      public:
        uint8_t fwd;
        uint8_t bwd;
        uint8_t gpio;
        Pins(const uint8_t _fwd, const uint8_t _bwd, const uint8_t _gpio);
    };

    class Pwm {
      public:
        uint8_t channel;
        const uint8_t resolution = MOTOR_PWM_RESOLUTION;
        const uint16_t frequency = MOTOR_PWM_FREQUENCY;
        Pwm(const uint8_t channel);
    };

  public:
    Pins pins;
    Pwm pwm;
    Motor(const uint8_t _fwd, const uint8_t _bwd, const uint8_t _gpio, const uint8_t _ch);
    void setup();
    void drive(int16_t velocity);
};

class Motors {
  public:
    Motor left;
    Motor right;
    Motors(Motor& _left, Motor& _right);
    void setup();
};

#endif
