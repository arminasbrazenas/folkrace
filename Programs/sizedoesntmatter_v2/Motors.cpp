#include "Motors.h"

Motor::Pins::Pins(const uint8_t _fwd, const uint8_t _bwd, const uint8_t _gpio)
  : fwd(_fwd), bwd(_bwd), gpio(_gpio) {};

Motor::Pwm::Pwm(const uint8_t _channel)
  : channel(_channel) {};

Motor::Motor(const uint8_t fwd, const uint8_t bwd, const uint8_t gpio, const uint8_t ch)
  : pins(fwd, bwd, gpio), pwm(ch) {};

void Motor::setup() {
  pinMode(this->pins.fwd, OUTPUT);
  pinMode(this->pins.bwd, OUTPUT);
  ledcAttachPin(this->pins.gpio, this->pwm.channel);
  ledcSetup(this->pwm.channel, this->pwm.frequency, this->pwm.resolution);
}

void Motor::drive(int16_t velocity) {  
  if (velocity > 0) {
    digitalWrite(this->pins.fwd, HIGH);
    digitalWrite(this->pins.bwd, LOW);
  } else  if (velocity < 0){
    digitalWrite(this->pins.fwd, LOW);
    digitalWrite(this->pins.bwd, HIGH);
  } else {
    digitalWrite(this->pins.fwd, LOW);
    digitalWrite(this->pins.bwd, LOW);
  }

  ledcWrite(this->pwm.channel, abs(velocity));
};

Motors::Motors(Motor& _left, Motor& _right)
  : left(_left), right(_right) {};

void Motors::setup() {
  this->left.setup();
  this->right.setup();
}
