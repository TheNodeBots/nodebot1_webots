// File:          ChassisSmallCollisionAvoidance.cpp
// Date:          2022-12-31
// Description:   Collision avoidance controller for the small robot chassis 'ChassisSmall'
// Author:        Stephan Kunz
// Modifications: Initial Setup

// add includes
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>

#include <string>
#include <cmath>

// Give access to the namespace of all the webots classes
using namespace webots;

// definition of min & max speed in m/s
#define MIN_SPEED 0.1
#define MAX_SPEED 3.0

// definition of max measurable distance in cm
#define MAX_DISTANCE 450.0
// definition of the distance when slowdown begins in cm
#define SLOWDOWN_DISTANCE 50.0
// definition of security distance in cm
#define SECURITY_DISTANCE 7.5
// definition of distance for angular sensors in cm
#define ANGULAR_DISTANCE 70.0
// definition of distance for side sensors in cm
#define SIDE_DISTANCE 10.0
// sensivity of the side sensors [-1, 1]
#define SENSIVITY 0.20

// location definitions fitting the arrays
#define FRONT  0
#define REAR   1
#define SIDE   2
#define LEFT   0
#define RIGHT  1
#define CENTER 2

class Sensor {
  public:
    Sensor(Robot *robot, const char *direction) {
      // get the time step of the current world.
      timeStep = (int)robot->getBasicTimeStep();
      std::string directionAsString = direction;

      ds = robot->getDistanceSensor(directionAsString + "DS");
    }

    void enable() {
      if(enabled)
        return;
      ds->enable(timeStep);
      enabled = true;
    }

    void disable() {
      if(!enabled)
        return;
      ds->disable();
      enabled = false;
    }

    bool isEnabled() {
      return enabled;
    }

    double getValue() {
      return std::min(ds->getValue(), SIDE_DISTANCE) / SIDE_DISTANCE;
    }

  private:
    int timeStep = 32;
    bool enabled = false;
    DistanceSensor *ds;
};


class SensorGroup {
  public:
    SensorGroup(Robot *robot, int direction) {
      // get the time step of the current world.
      timeStep = (int)robot->getBasicTimeStep();

      if(direction == FRONT) {
        directionAsString = "Front";
      } else {
        directionAsString = "Rear";
      }

      ds[LEFT]   = robot->getDistanceSensor(directionAsString + "LeftDS");
      ds[CENTER] = robot->getDistanceSensor(directionAsString + "CenterDS");
      ds[RIGHT]  = robot->getDistanceSensor(directionAsString + "RightDS");
    }

    void enable() {
      if(enabled)
        return;
      ds[LEFT]->enable(timeStep);
      ds[CENTER]->enable(timeStep);
      ds[RIGHT]->enable(timeStep);
      enabled = true;
    }

    void disable() {
      if(!enabled)
        return;
      ds[LEFT]->disable();
      ds[CENTER]->disable();
      ds[RIGHT]->disable();
      enabled = false;
    }

    bool isEnabled() {
      return enabled;
    }

    double getValue(int i) {
      return std::min(SLOWDOWN_DISTANCE, ds[i]->getValue() - SECURITY_DISTANCE) / SLOWDOWN_DISTANCE;
    }

    double getMinValue() {
      double minVal = std::min(ds[CENTER]->getValue(), ds[LEFT]->getValue());
      minVal = std::min(minVal, ds[RIGHT]->getValue());
      return std::min(SLOWDOWN_DISTANCE, minVal - SECURITY_DISTANCE) / SLOWDOWN_DISTANCE;
    }

   double getSideValue(int i) {
      return std::max(0.0, ANGULAR_DISTANCE - ds[i]->getValue()) / ANGULAR_DISTANCE;
    }

    void printValues() {
      if(enabled) {
        std::cout << directionAsString << "Left: "   <<  ds[LEFT]  ->getValue() << std::endl;
        std::cout << directionAsString << "Center: " <<  ds[CENTER]->getValue() << std::endl;
        std::cout << directionAsString << "Right: "  <<  ds[RIGHT] ->getValue() << std::endl;
      }
    }

  private:
    int timeStep = 32;
    bool enabled = false;
    // distance sensors
    DistanceSensor *ds[3];
    std::string directionAsString;
};

class MotorController {
  public:
    MotorController(Robot *robot, int side) {
      std::string sideAsString;

      if(side == LEFT) {
        sideAsString = "Left";
        max_speed = 3.0;
      } else {
        sideAsString = "Right";
        max_speed = -3.0;
      }

      front = robot->getMotor("Front" + sideAsString + "Motor");
      rear  = robot->getMotor("Rear" + sideAsString + "Motor");
      front->setPosition(INFINITY);
      rear ->setPosition(INFINITY);
    }

    void setSpeed(float value) {
      front->setVelocity(max_speed * value);
      rear ->setVelocity(max_speed * value);
    }

  private:
    Motor *front, *rear;
    float max_speed;
};

// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  float wantedSpeed = MAX_SPEED;
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // distance sensors
  SensorGroup *frontSensors, *rearSensors;
  Sensor *leftSensor, *rightSensor;

  // motor controllers
  MotorController *controller[2];
  
  // initialize everything
  frontSensors = new SensorGroup(robot, FRONT);
  rearSensors = new SensorGroup(robot, REAR);
  leftSensor = new Sensor(robot, "Left");
  rightSensor = new Sensor(robot, "Right");

  controller[LEFT]  = new MotorController(robot, LEFT);
  controller[RIGHT] = new MotorController(robot, RIGHT);

  float leftSpeed, rightSpeed;
  float oldLeftSpeed, oldRightSpeed;

  leftSpeed = rightSpeed = wantedSpeed;
  oldLeftSpeed = oldRightSpeed = 0.0;
  
  // start with all sensors enabled
  // side sensors should always be enabled, front and rear dependant on direction of moving
  leftSensor  ->enable();
  rightSensor ->enable();
  frontSensors->enable();
  rearSensors ->enable();

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // enable control outputs for debugging
//    frontSensors->printValues();
//    rearSensors ->printValues();
    // Read the sensors
    // Process sensor data
    if(leftSpeed + rightSpeed > (2 * MIN_SPEED)) {
      frontSensors->enable();
      rearSensors ->disable();
      leftSpeed  = (oldLeftSpeed  + wantedSpeed * (frontSensors->getMinValue() + SENSIVITY*frontSensors->getSideValue(LEFT) - SENSIVITY*frontSensors->getSideValue(RIGHT) + leftSensor->getValue() - rightSensor->getValue())) / 4.0;
      rightSpeed = (oldRightSpeed + wantedSpeed * (frontSensors->getMinValue() - SENSIVITY*frontSensors->getSideValue(LEFT) + SENSIVITY*frontSensors->getSideValue(RIGHT) - leftSensor->getValue() + rightSensor->getValue())) / 4.0;
    } else if(leftSpeed + rightSpeed < -(2 * MIN_SPEED)) {
      frontSensors->disable();
      rearSensors ->enable();
      leftSpeed  = (oldLeftSpeed  + wantedSpeed * (rearSensors->getMinValue() + SENSIVITY*rearSensors->getSideValue(LEFT) - SENSIVITY*rearSensors->getSideValue(RIGHT) - leftSensor->getValue() + rightSensor->getValue())) / 4.0;
      rightSpeed = (oldRightSpeed + wantedSpeed * (rearSensors->getMinValue() - SENSIVITY*rearSensors->getSideValue(LEFT) + SENSIVITY*rearSensors->getSideValue(RIGHT) + leftSensor->getValue() - rightSensor->getValue())) / 4.0;
    } else {
      frontSensors->enable();
      rearSensors ->enable();
      wantedSpeed = -wantedSpeed;
      leftSpeed  = wantedSpeed;
      rightSpeed = wantedSpeed;
      oldLeftSpeed = oldRightSpeed = 0.0;
    }

    // Send actuator commands.
    controller[LEFT] ->setSpeed(leftSpeed);
    controller[RIGHT]->setSpeed(rightSpeed);

    oldLeftSpeed = leftSpeed;
    oldRightSpeed = rightSpeed;
  };

  // cleanup code.
  delete robot;
  return 0;
}
