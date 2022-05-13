#include <PID_v1.h>
#include <pidautotuner.h>
#include <BluetoothSerial.h>
#include "Motors.h"
#include "DistanceSensors.h"
#include "ServoSteering.h"
#include "Constants.h"

// Initialize motors
Motor leftMotor(LEFT_MOTOR_FWD_PIN, LEFT_MOTOR_BWD_PIN, LEFT_MOTOR_GPIO_PIN, LEFT_MOTOR_PWM_CHANNEL);
Motor rightMotor(RIGHT_MOTOR_FWD_PIN, RIGHT_MOTOR_BWD_PIN, RIGHT_MOTOR_GPIO_PIN, RIGHT_MOTOR_PWM_CHANNEL);
Motors motors(leftMotor, rightMotor);

// Initialize distance sensors
DistanceSensors distanceSensors(7, (uint8_t[]){ 27, 26, 25, 33, 32, 35, 34 });

// Initialize servo
ServoSteering servo;

// Define variables we'll be connecting steering PID to
double setpoint, input, output;

// Specify steering PID links and initial tuning parameters
double kp = 0.1, ki = 0, kd = 0;
PID steeringPID(&input, &output, &setpoint, kp, ki, kd, REVERSE);

// Initialize bluetooth
BluetoothSerial SerialBT;

String receivedData = "";

void evaluateReceivedData(String& action, float& value) {
  action = "";
  String valueStr = "";
  bool actionFound = false;

  for (int i = 0; i < receivedData.length(); i++) {
    if (!actionFound && !isDigit(receivedData[i])) {
      action += receivedData[i];
    }

    if (isDigit(receivedData[i]) || receivedData[i] == '.') {
      actionFound = true;
      valueStr += receivedData[i];
    }
  }

  value = valueStr.toFloat();
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("sdm"); // Bluetooth device name
  servo.setup(SERVO_GPIO_PIN);
  motors.setup();
  distanceSensors.setup();
  servo.write(SERVO_POS_MID);

  input = distanceSensors.getDifference();
  setpoint = 0;
  steeringPID.SetOutputLimits(SERVO_POS_MIN, SERVO_POS_MAX);
  steeringPID.SetMode(AUTOMATIC);
}

bool isRobotStarted = false;
int16_t robotSpeed = 0;

unsigned long startMs = 0;
int startDelay = 0;

unsigned long frontObstacleStartMs = 0;
bool obstacleInFront = false;

void loop() {
  while (SerialBT.available()) {
    char receivedChar = SerialBT.read();

    if (receivedChar == '#') {
      String action;
      float value;
      evaluateReceivedData(action, value);

      if (action == "START"){
        isRobotStarted = true;
        startMs = millis();
      } else if (action == "STOP") {
        isRobotStarted = false;
        robotSpeed = 0;
      } else if (action == "SPEED") {
        robotSpeed = (int)value;
      } else if (action == "DELAY") {
        startDelay = (int)value * 1000;
      } else if (action == "P") {
        kp = value;
      } else if (action == "I") {
        ki = value;
      } else if (action == "D") {
        kd = value;
      }

      if (action == "P" || action == "I" || action == "D") {
        steeringPID.SetTunings(kp, ki, kd);
        steeringPID.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
        steeringPID.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
        steeringPID.SetOutputLimits(SERVO_POS_MIN, SERVO_POS_MAX);  // Set the limits back to normal
      }

      receivedData = "";
    } else {
      receivedData += receivedChar;
    }
  }

  if (isRobotStarted && startMs + startDelay <= millis()) {
    motors.left.drive(robotSpeed);
    motors.right.drive(robotSpeed);
    input = distanceSensors.getDifference();
    steeringPID.Compute();

    servo.write(output);

    /*
    // If middle sensor detects an obstacle in front
    if (distanceSensors.sensors[3].getDistance() <= 150) {
      // If obstacle in front is detected for the first time
      if (!obstacleInFront) {
        frontObstacleStartMs = millis();
        obstacleInFront = true;
      }

      // If obstacle in front has been there for a certain amount of time, go back
      if (obstacleInFront && millis() - frontObstacleStartMs >= 1000) {
        servo.write(SERVO_POS_MID);
        motors.left.drive(-100);
        motors.right.drive(-100);
        delay(800);

        obstacleInFront = false;
        frontObstacleStartMs = 0;
      } else {
        obstacleInFront = false;
        frontObstacleStartMs = 0;
      }
    }*/
  } else {
    motors.left.drive(0);
    motors.right.drive(0);
    servo.write(SERVO_POS_MID);
  }
}
