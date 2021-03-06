
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>

#define TIME_STEP 64
using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  Keyboard kb;
  DistanceSensor *ds[2];
  
 
  Motor *wheels[4];
  char wheels_names[4][32] = {"wheel_motor_front_left", "wheel_motor_front_right", "wheel_motor_rear_right", "wheel_motor_rear_left"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  
  kb.enable(TIME_STEP);
  double leftSpeed = 10.0;
  double rightSpeed = 10.0;
  while (robot->step(TIME_STEP) != -1) {
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