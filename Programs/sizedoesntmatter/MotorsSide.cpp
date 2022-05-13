#include "MotorsSide.h"

void MotorsSide::setPins(uint8_t fwdPin, uint8_t bwdPin) {
  this->fwdPin = fwdPin;
  this->bwdPin = bwdPin;

  pinMode(fwdPin, OUTPUT);
  pinMode(bwdPin, OUTPUT);
}

void MotorsSide::drive(int16_t motorSpeed) {
  if (motorSpeed > 0) {
    if (motorSpeed > 255) {
      motorSpeed = 255; 
    }
      
    analogWrite(fwdPin, motorSpeed);
    analogWrite(bwdPin, 0);
  } else {
    if (motorSpeed < -255) {
      motorSpeed = -255;
    }
    
    analogWrite(fwdPin, 0);
    analogWrite(bwdPin, abs(motorSpeed));
  }
}
