#include <agv/motor_controller.h>
#include "wiringPi.h"
#include <softPwm.h> 

#define LEFT_ENABLE 6
#define RIGHT_ENABLE 23
#define LEFT_FORWARD 21
#define LEFT_BACKWARD 22
#define RIGHT_FORWARD 26
#define RIGHT_BACKWARD 27

MotorController::MotorController()
{
  if(wiringPiSetup() == -1){
    exit(1);
  }
  pinMode(LEFT_ENABLE, OUTPUT);
  pinMode(RIGHT_ENABLE, OUTPUT);
  pinMode(LEFT_FORWARD, OUTPUT);
  pinMode(LEFT_BACKWARD, OUTPUT);
  pinMode(RIGHT_FORWARD, OUTPUT);
  pinMode(RIGHT_BACKWARD, OUTPUT);
  softPwmCreate(LEFT_ENABLE,0,100);
  softPwmCreate(RIGHT_ENABLE,0,100);
}

MotorController::~MotorController()
{
  softPwmStop(LEFT_ENABLE);
  softPwmStop(RIGHT_ENABLE);
}

void MotorController::setDirection(const geometry_msgs::msg::Twist::SharedPtr dir) const
{
  int rightPower = (dir->linear.x * 100) - (dir->linear.x * 100 * dir->angular.z);
  int leftPower = dir->linear.x * 100 - (dir->linear.x * 100 * (0 - dir->angular.z));
  if(dir->linear.x >= 0.2){
    goForward();
    softPwmWrite (RIGHT_ENABLE, rightPower) ;
    softPwmWrite (LEFT_ENABLE, leftPower) ;
  } else if(dir->linear.x <= -0.2){
    goBackward();
    softPwmWrite (RIGHT_ENABLE, abs(leftPower)) ;
    softPwmWrite (LEFT_ENABLE, abs(rightPower)) ;
  } else{
    softPwmWrite (RIGHT_ENABLE, 0) ;
    softPwmWrite (LEFT_ENABLE, 0) ;
  }
  // delay(500);
  // softPwmWrite(RIGHT_ENABLE, 0);
  // softPwmWrite(LEFT_ENABLE, 0);
}

void MotorController::goBackward() const{
  digitalWrite(LEFT_FORWARD, LOW);
  digitalWrite(RIGHT_FORWARD, LOW);
  digitalWrite(LEFT_BACKWARD, HIGH);
  digitalWrite(RIGHT_BACKWARD, HIGH);
}

void MotorController::goForward() const {
  digitalWrite(LEFT_FORWARD, HIGH);
  digitalWrite(RIGHT_FORWARD, HIGH);
  digitalWrite(LEFT_BACKWARD, LOW);
  digitalWrite(RIGHT_BACKWARD, LOW);
}