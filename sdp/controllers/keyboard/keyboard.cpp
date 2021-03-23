
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>

#include <iostream>
#include <sstream>

#define TIME_STEP 64
using namespace webots;
using namespace std;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  Keyboard kb;
  
 
  Motor *wheels[4];
  char wheels_names[4][32] = {"wheel_motor_front_left", "wheel_motor_front_right", "wheel_motor_rear_right", "wheel_motor_rear_left"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  
  DistanceSensor *ps[3];
  //Sharp's IR sensor GP2Y0A710K0F
  char psNames[3][32] = {"distance_sensor_left","distance_sensor_front","distance_sensor_right"};
  for (int i = 0; i < 3; i++) {
  ps[i] = robot->getDistanceSensor(psNames[i]);
  ps[i]->enable(TIME_STEP);
}

  kb.enable(TIME_STEP);
  double leftSpeed = 10.0;
  double rightSpeed = 10.0;
  
  while (robot->step(TIME_STEP) != -1) {
    double psValues[3];
        for (int i = 0; i < 3 ; i++)
          psValues[i] = ps[i]->getValue();
    
    cout << "Left: "<< psValues[0]<<" Front: "<< psValues[1] <<" Right: "<< psValues[2] << " "<< endl;
    
    int key=kb.getKey();
    
    if (key==315){
    leftSpeed = 10.0;
    rightSpeed = 10.0;
    } else if (key==317){
    leftSpeed = -10.0;
    rightSpeed = -10.0;
    }else if (key==316){
    leftSpeed = 10.0;
    rightSpeed = -10.0;
    }else if (key==314){
    leftSpeed = -10.0;
    rightSpeed = 10.0;
    }else {
    leftSpeed = 0.0;
    rightSpeed = 0.0;
    }
    
    wheels[0]->setVelocity(leftSpeed);
    wheels[1]->setVelocity(rightSpeed);
    wheels[2]->setVelocity(rightSpeed);
    wheels[3]->setVelocity(leftSpeed);
    
      }
  delete robot;
  return 0;  // EXIT_SUCCESS
}