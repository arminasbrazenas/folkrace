#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "Arduino.h"

const uint8_t MOTOR_PWM_RESOLUTION = 8;
const uint16_t MOTOR_PWM_FREQUENCY = 30000;

const uint8_t LEFT_MOTOR_FWD_PIN = 21;
const uint8_t LEFT_MOTOR_BWD_PIN = 22;
const uint8_t LEFT_MOTOR_GPIO_PIN = 23;
const uint8_t LEFT_MOTOR_PWM_CHANNEL = 5;

const uint8_t RIGHT_MOTOR_FWD_PIN = 5;
const uint8_t RIGHT_MOTOR_BWD_PIN = 19;
const uint8_t RIGHT_MOTOR_GPIO_PIN = 18;
const uint8_t RIGHT_MOTOR_PWM_CHANNEL = 6;

const uint8_t SERVO_GPIO_PIN = 12;
const uint8_t SERVO_POS_MIN = 50;
const uint8_t SERVO_POS_MID = 85;
const uint8_t SERVO_POS_MAX = 120;

#endif
