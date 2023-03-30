//Put the home switch detection code below

#include "ClearCore.h"

#define MOTOR_AT_HOME 1
#define MOTOR_NOT_AT_HOME 2
#define Home_pin A9 //Connect homing probe to DI7

int HomeSensorState;
void detectHomeSensorState();

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
  attachInterrupt(digitalPinToInterrupt(Home_pin), detectHomeSensorState, RISING);

  // Enable digital interrupts.
  interrupts();
}


//Put the bladesaw's state based on switch on and off on ClearCore
//SET MOTOR_AT_HOME or MOTOR_NOT_AT_HOME
void detectHomeSensorState() {

// Read the state of the limit switch connected to digital pin 7
  int switchState = analogRead(Home_pin);
  // If the switch is  triggered, set Motr at home
  if (switchState < 2800) {
    motor.MoveStopAbrupt();
    motor.PositionRefSet(0);
    Serial.print("Homed - sensor, triggered at "); Serial.print(switchState);
    HomeSensorState = MOTOR_AT_HOME;
    delay(2000); //Testing, change to 50
  }
  // If the switch is not triggered, motor is not home
  else {
    HomeSensorState = MOTOR_NOT_AT_HOME;
  }
 
}

void UserSeeksHome(void){//Check step direction, whether clockwise or anticlockwise is toward blade
  /* Move towards the blade for homing, then repeat much more slowly to prevent blade deflection. Needs to be adjusted to use limit switch as probe */
    // Commands a speed of 10000 pulses/sec away from blade for 0.5 seconds
    Serial.println("Homing . . . Waiting for motor to finish");
    motor.MoveVelocity(10000);//Move away from blade
    delay(500);
    motor.MoveVelocity(-9000);//Move towards blade
    Serial.println("set vel to -9000, homing");
    
    delay(500);
    detectMotorStates(0);
    
    while (MotorRunState != MOTOR_STOPPED || motor.HlfbState() != MotorDriver::HLFB_ASSERTED) 
    {              
      
      detectHomeSensorState();
      detectMotorStates(0);
          
      continue;
    }

    //Back out after touching the probe, like 3D-printer homing
    motor.Move(6400); 
    delay(500);
    motor.MoveVelocity(-800); //0.125 revolutions per second

    delay(500);
    detectMotorStates(0);

    while (MotorRunState != MOTOR_STOPPED || motor.HlfbState() != MotorDriver::HLFB_ASSERTED) 
    {              
      
      detectHomeSensorState();
      detectMotorStates(0);
          
      continue;
    }
    
    // Then slows down to 3200 pulses/sec until clamping into the hard stop
    motor.MoveVelocity(-3200);
    // Delay so HLFB has time to deassert
//    delay(10);
    // Waits for HLFB to assert again, meaning the hardstop has been reached
    while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        continue;
    }
    // Stop the velocity move now that the hardstop is reached
    motor.MoveStopAbrupt();
    motor.PositionRefSet(0);
    Serial.println("Homed - UserSeeksHome(), triggered by HLFB assertion (hardstop)");

    return;
}