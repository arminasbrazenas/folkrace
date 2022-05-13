#include "DistanceSensors.h"

void DistanceSensor::setPin(const uint8_t _pin) {
  pin = _pin;
}

void DistanceSensor::setup() {
  pinMode(pin, INPUT);
}

// Returns measured distance in millimeters
int32_t DistanceSensor::getDistance() {
  int16_t timeInMicroseconds = pulseIn(pin, HIGH);
  const int16_t MAX_DISTANCE = 800;

  if (timeInMicroseconds == 0) {
    // pulseIn() did not detect the start of a pulse within 1 second.
    return MAX_DISTANCE;
  }
  else if (timeInMicroseconds > 1850) {
    // No detection.
    return MAX_DISTANCE;
  }
  else {
    // Valid pulse width reading. Convert pulse width in microseconds to distance in millimeters.
    int16_t distanceInMillimeters = (timeInMicroseconds - 1000) * 2;

    // Limit minimum distance to 0.
    if (distanceInMillimeters < 0) {
      distanceInMillimeters = 0;
    }

    if (distanceInMillimeters > MAX_DISTANCE) {
      distanceInMillimeters = MAX_DISTANCE;
    }

    return distanceInMillimeters;
  }
}

DistanceSensors::DistanceSensors(uint8_t _total, uint8_t pins[]) {
  total = _total;

  for (uint8_t i = 0; i < total; i++) {
    sensors[i].setPin(pins[i]);
  }
}

void DistanceSensors::setup() {
  for (uint8_t i = 0; i < total; i++) {
    sensors[i].setup();
  }
}

int32_t DistanceSensors::getDifference() {
  int32_t distanceLeft = (this->sensors[0].getDistance() + this->sensors[1].getDistance() + this->sensors[2].getDistance());
  int32_t distanceRight = (this->sensors[6].getDistance() + this->sensors[5].getDistance() + this->sensors[4].getDistance());
  
  int32_t diff = distanceLeft - distanceRight;

  return diff;
}
