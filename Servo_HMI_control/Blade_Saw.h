/* 
 *  Assumes normally-closed blade state switch
*/
#include "ClearCore.h"

#define BLADE_UP 7
#define BLADE_DOWN 8
#define Blade_State_Pin DI6

int BladeState;

void setBladeState(int state) {
  BladeState = state;
}

int getBladeState() {
  return BladeState;
}


//Put the bladesaw's state based on switch on and off on ClearCore
//SET BLADE_UP or BLADE_DOWN
void detectBladeState() {
  // Read the state of the limit switch connected to digital I/O 16
  int switchState = digitalRead(Blade_State_Pin);

  // If the switch is not triggered, set BladeState to BLADE_UP
  if (switchState == LOW) {
    setBladeState(BLADE_UP);
  }
  // If the switch is triggered, set BladeState to BLADE_DOWN
  else {
    setBladeState(BLADE_DOWN);
  }
}