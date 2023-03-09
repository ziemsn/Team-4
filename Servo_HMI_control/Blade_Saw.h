

#include "ClearCore.h"

#define BLADE_UP 7
#define BLADE_DOWN 8

int BladeState;

void setBladeState(int state) {
  BladeState = state;
}

int getBladeState() {
  return BladeState;
}


//Put the bladesaw's state based on switch on and off on ClearCore
//Return BLADE_UP or BLADE_DOWN
int detectBladeState() {


  
}
