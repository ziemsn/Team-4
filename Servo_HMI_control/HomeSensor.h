//Put the home switch detection code below

#include "ClearCore.h"

#define MOTOR_AT_HOME 1
#define MOTOR_NOT_AT_HOME 2


int HomeSensorState;

void setHomeSensorState(int state) {
  HomeSensorState = state;
}

int getHomeSensorState() {
  return HomeSensorState;
}


//Put the bladesaw's state based on switch on and off on ClearCore
//SET MOTOR_AT_HOME or MOTOR_NOT_AT_HOME
void detectHomeSensorState() {


 
}
