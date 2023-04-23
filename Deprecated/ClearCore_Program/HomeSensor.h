#include "ClearCore.h"

#define MOTOR_AT_HOME 1
#define MOTOR_NOT_AT_HOME 2
#define Home_pin A9 //Connect homing probe to A9

int HomeSensorState;

// Function to detect the state of the home sensor switch
void detectHomeSensorState();

// Function to set the state of the home sensor switch
void setHomeSensorState() {
  // Read the state of the limit switch connected to analog pin 9
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

// Function to get the state of the home sensor switch
int getHomeSensorState() {
  return HomeSensorState;
}

// Function to initialize homing
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
    Serial.print("Homed - sensor, triggered at "); Serial.println(switchState);
    HomeSensorState = MOTOR_AT_HOME;
    delay(1000); //Testing, change to 50
  }

  // If the switch is not triggered, motor is not home
  else {
    HomeSensorState = MOTOR_NOT_AT_HOME;
  }
 
}

// Function to home the motor
void UserSeeksHome(void){//Check step direction, whether clockwise or anticlockwise is toward blade
  /* Move towards the blade for homing, then repeat much more slowly to prevent blade deflection. Needs to be adjusted to use limit switch as probe */
    // Commands a speed of 10000 pulses/sec away from blade for 0.5 seconds

    Serial.println("Homing . . . Waiting for motor to finish");
    motor.MoveVelocity(10000);//Move away from blade
    delay(500);
    motor.MoveVelocity(-12000);//Move towards blade
    Serial.println("homing");
    
    delay(100);
    detectMotorStates(0);
    
    // Wait for the motor to stop and for the HLFB state to be asserted
    while (MotorRunState != MOTOR_STOPPED || motor.HlfbState() != MotorDriver::HLFB_ASSERTED) 
    {              
      
      detectHomeSensorState();
      detectMotorStates(0);
          
      continue;
    }

    //Back out after touching the probe, like 3D-printer homing
    motor.MoveVelocity(6400); 
    delay(500);
    motor.MoveVelocity(-400); //0.0625 revolutions per second

    delay(100);
    detectMotorStates(0);

    // Wait for the motor to stop and for the HLFB state to be asserted
    while (MotorRunState != MOTOR_STOPPED || motor.HlfbState() != MotorDriver::HLFB_ASSERTED) 
    {              
      
      detectHomeSensorState();
      detectMotorStates(0);
          
      continue;
    }
    
    return;
}