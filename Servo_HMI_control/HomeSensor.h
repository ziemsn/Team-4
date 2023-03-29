//Put the home switch detection code below

#include "ClearCore.h"

#define MOTOR_AT_HOME 1
#define MOTOR_NOT_AT_HOME 2
#define Home_pin A9 //Connect homing probe to DI7

int HomeSensorState;

void setHomeSensorState() {
  // Read the state of the limit switch connected to digital pin 7
  int switchState = analogRead(Home_pin);
  Serial.println(switchState);
  // If the switch is  triggered, set Motr at home
  if (switchState == LOW) {
    motor.MoveStopAbrupt();
    motor.PositionRefSet(0);
    Serial.println("Homed - interrupt");
    HomeSensorState = MOTOR_AT_HOME;
    
  }
  // If the switch is not triggered, motor is not home
  else {
    HomeSensorState = MOTOR_NOT_AT_HOME;
  }
}

int getHomeSensorState() {
  return HomeSensorState;
}

void InitHoming(){
  // Set up the interrupt pin in digital input mode.
  pinMode(Home_pin, INPUT);

  // Set an ISR to be called when the state of the interrupt pin goes from
  // true to false.
  attachInterrupt(digitalPinToInterrupt(Home_pin), setHomeSensorState, RISING);

  // Enable digital interrupts.
  interrupts();
}


//Put the bladesaw's state based on switch on and off on ClearCore
//SET MOTOR_AT_HOME or MOTOR_NOT_AT_HOME
void detectHomeSensorState() {

// Read the state of the limit switch connected to digital pin 7
  int switchState = analogRead(Home_pin);
  // If the switch is  triggered, set Motr at home
  if (switchState < 500) {
    motor.MoveStopAbrupt();
    motor.PositionRefSet(0);
    Serial.print("Homed - sensor, triggered at "); Serial.print(switchState);
    HomeSensorState = MOTOR_AT_HOME;
    
  }
  // If the switch is not triggered, motor is not home
  else {
    HomeSensorState = MOTOR_NOT_AT_HOME;
  }
 
}