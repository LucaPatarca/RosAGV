#include <agv/clamp_controller.h>
#include "wiringPi.h"
#include <softPwm.h> 

#define CLAMP_PIN 1

ClampController::ClampController()
{
  if(wiringPiSetup() == -1){
    exit(1);
  }
  pinMode(CLAMP_PIN, OUTPUT);
  softPwmCreate(CLAMP_PIN, 0, 50);
}

ClampController::~ClampController()
{
  softPwmStop(CLAMP_PIN);
}

void ClampController::setPosition(const double value) const
{
  softPwmWrite(1, 6 + (value * 8));
}