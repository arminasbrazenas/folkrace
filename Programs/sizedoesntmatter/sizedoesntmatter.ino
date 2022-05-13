#include <SoftwareSerial.h>
#include <PID_v2.h>
#include "SharpIRSensor.h"
#include "MotorsSide.h"

SoftwareSerial bluetooth(2, 4);

class IRSensors
{
  public:
    SharpIRSensor left;
    SharpIRSensor middle;
    SharpIRSensor right;
    uint16_t distanceLimit = 800; // In millimeters
    int16_t sideDifference;
    void measureSideDifference(); 
};

void IRSensors::measureSideDifference()
{
  uint16_t distanceLeft = right.getDistance();
  uint16_t distanceRight = left.getDistance();

  if (distanceLeft > distanceLimit) {
    distanceLeft = distanceLimit;
  }
  if (distanceRight > distanceLimit) {
    distanceRight = distanceLimit;
  }

  sideDifference = distanceLeft - distanceRight;
}

class Motors
{
  public:
    MotorsSide left;
    MotorsSide right;
    uint8_t maxSpeed = 0;
};

class Robot
{
  public:
    IRSensors sensors;
    Motors motors;
    uint8_t state = 0;
    uint16_t startDelay = 5000;
    bool hasCollided = false;
    uint32_t millisAtCollision = 0;
};

Robot robot;

double Kp = 8, Ki = 10, Kd = 1;
PID_v2 steeringPID(Kp, Ki, Kd, PID::Direct);

uint32_t startMillis = 0;

void setup()
{
  // Begin serial communication
  Serial.begin(9600);
  bluetooth.begin(9600);

  // Set IR sensor pins
  robot.sensors.right.setPin(A2);
  robot.sensors.middle.setPin(A3);
  robot.sensors.left.setPin(A4);

  // Set IR sensor power function coefficients
  robot.sensors.right.setFunctionCoeffs(965212, -1.413);
  robot.sensors.middle.setFunctionCoeffs(477335, -1.312);
  robot.sensors.left.setFunctionCoeffs(986686, -1.416);

  // Set motor pins (FWD and BWD)
  robot.motors.left.setPins(5, 3);
  robot.motors.right.setPins(6, 9);

  // Configure PID for steering
  robot.sensors.measureSideDifference();
  steeringPID.Start(abs(robot.sensors.sideDifference), robot.motors.maxSpeed, 0);
  steeringPID.SetOutputLimits(0, robot.motors.maxSpeed);
  steeringPID.SetSampleTime(20);
}

void loop()
{
  const uint32_t currentMillis = millis();
  
  if (bluetooth.available()) {
    robot.state = bluetooth.read();
    delay(20);

    if (robot.state == 0) {
      robot.motors.maxSpeed = 0;
    } else if (robot.state == 1) {
      startMillis = currentMillis;
      
      robot.motors.maxSpeed = bluetooth.read();
      delay(20);
      robot.sensors.distanceLimit = bluetooth.read();
      delay(20);
      robot.startDelay = bluetooth.read();
      delay(20);
      Kp = bluetooth.read();
      delay(20);
      Ki = bluetooth.read();
      delay(20);
      Kd = bluetooth.read();
      delay(20);

      robot.sensors.distanceLimit *= 10;
      robot.startDelay *= 1000;
      Kp /= 10;
      Ki /= 10;
      Kd /= 10;

      steeringPID.SetOutputLimits(0.0, 1.0);
      steeringPID.SetOutputLimits(-1.0, 0.0);
      steeringPID.SetOutputLimits(0, robot.motors.maxSpeed);
      steeringPID.SetTunings(Kp, Ki, Kd);
    } else if (robot.state == 2) {
      robot.motors.left.drive(255);
      robot.motors.right.drive(-255);
      delay(170);
    }
  }

  if (robot.motors.maxSpeed != 0 && (currentMillis - startMillis >= robot.startDelay)) {
    robot.sensors.measureSideDifference();

    const uint8_t output = steeringPID.Run(abs(robot.sensors.sideDifference));
    if (robot.sensors.sideDifference > 0) {
      robot.motors.left.drive(output);
      robot.motors.right.drive(robot.motors.maxSpeed);
    } else {
      robot.motors.left.drive(robot.motors.maxSpeed);
      robot.motors.right.drive(output);
    }

    // Check for collision
    if (robot.sensors.middle.getDistance() <= 150) {
       robot.hasCollided = true;

       if (robot.millisAtCollision == 0) {
        robot.millisAtCollision = currentMillis;
       }

       if (currentMillis - robot.millisAtCollision >= 1000) {
        robot.motors.left.drive(-100);
        robot.motors.right.drive(-100);
        delay(500);
       }
    } else {
        robot.hasCollided = false;
        robot.millisAtCollision = 0;
    }
  } else {
    robot.motors.left.drive(0);
    robot.motors.right.drive(0);
  }
}
