#include "ClearCore.h"

// Specifies which motor to move.
// Options are: ConnectorM0, ConnectorM1, ConnectorM2, or ConnectorM3.
#define motor ConnectorM0

// Select the baud rate to match the target serial device
#define baudRate 9600

// Define the velocity and acceleration limits to be used for each move
int velocityLimit = 8000; // pulses per sec
int homingVelocityLimit = 4000; // pulses per sec
int accelerationLimit = 16000; // pulses per sec^2
int homingAccelerationLimit = 4000; // pulses per sec^2

// Define Distance per Pulses
double inchPerPulse = 0.000147675662992126; // inches per pulse
double mmPerPulse = 0.00375096184; // mm per pulse

//Declare variables used in position calculation
int pulsesToMoveInches;
int pulsesToMoveMM;
double distanceToMoveInches;
double distanceToMoveMM;

// Declares our user-defined helper function, which is used to command moves to
// the motor. The definition/implementation of this function is at the  bottom
// of the example
bool MoveAbsolutePosition(int32_t position);

//Define an interrupt pin
#define interruptPin DI6

// Declare the signature for our interrupt service routine (ISR). The function
// is defined below
void Motorstop();

void setup() {
  // put your setup code here, to run once:
  
    
  // Sets the input clocking rate. This normal rate is ideal for ClearPath
  // step and direction applications.
  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

  // Sets all motor connectors into step and direction mode.
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                        Connector::CPM_MODE_STEP_AND_DIR);

  // Set the motor's HLFB mode to bipolar PWM
  motor.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
  // Set the HFLB carrier frequency to 482 Hz
  motor.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

  // Sets the maximum velocity for each move
  motor.VelMax(velocityLimit);

  // Set the maximum acceleration for each move
  motor.AccelMax(accelerationLimit);

  // Sets up serial communication and waits up to 5 seconds for a port to open.
  // Serial communication is not required for this example to run.
  Serial.begin(baudRate);
  uint32_t timeout = 5000;
  uint32_t startTime = millis();
  while (!Serial && millis() - startTime < timeout) {
      continue;
  }

  // Enables the motor; homing will begin automatically if enabled
  motor.EnableRequest(true);
  Serial.println("Motor Enabled");

  // Waits for HLFB to assert (waits for homing to complete if applicable)
  Serial.println("Waiting for HLFB...");
  while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
      continue;
      
  }
  Serial.println("Motor Ready");

  // Set up the interrupt pin in digital input mode.
  pinMode(interruptPin, INPUT);

  // Set an ISR to be called when the state of the interrupt pin goes from
  // true to false.
  attachInterrupt(digitalPinToInterrupt(interruptPin), Motorstop, RISING);

  // Enable digital interrupts.
  interrupts();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  distanceToMoveMM = PositionInputPrompt();

  pulsesToMoveMM = distanceToMoveMM / mmPerPulse;

  MoveAbsolutePosition(pulsesToMoveMM);
  delay(1000);

  MotorHoming();
  delay(1000);
  
}

double PositionInputPrompt() {

  
  // Ask for a position in MM to move slider to
  Serial.println("Enter desired position in MM:");
  while (Serial.available() == 0){}   // wait for input from user
  String positionString = Serial.readString();    // read in a value from user
  positionString.trim();
  Serial.print("Your entered position is ");
  Serial.print(positionString);
  Serial.println(" MM.");
  double positionMM = positionString.toDouble();    // convert input string to integer

  // comparison check if input is integer
  if (positionMM < 0 || positionMM > 280) {
    Serial.println("Out of range. Range is 0 mm to 280 mm");
    PositionInputPrompt();
  }
  else{
    Serial.print("Moving to "); 
    Serial.print(positionMM);
    Serial.println(" MM.");
    return positionMM;
  }
}

/*------------------------------------------------------------------------------
 * MoveAbsolutePosition
 *
 *    Command step pulses to move the motor's current position to the absolute
 *    position specified by "position"
 *    Prints the move status to the USB serial port
 *    Returns when HLFB asserts (indicating the motor has reached the commanded
 *    position)
 *
 * Parameters:
 *    int position  - The absolute position, in step pulses, to move to
 *
 * Returns: True/False depending on whether the move was successfully triggered.
 */
bool MoveAbsolutePosition(int position) {
    // Check if an alert is currently preventing motion
    if (motor.StatusReg().bit.AlertsPresent) {
        Serial.println("Motor status: 'In Alert'. Move Canceled.");
        return false;
    }

    
    Serial.print("Moving to absolute position: ");
    Serial.println(position);

    // Command the move of absolute distance
    motor.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);

    // Waits for HLFB to assert (signaling the move has successfully completed)
    Serial.println("Moving.. Waiting for HLFB");
    while (!motor.StepsComplete() || motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        continue;
    }

    Serial.println("Move Done");
    return true;
}

void Motorstop() {
    motor.MoveStopAbrupt();
    motor.PositionRefSet(0);  //resets home position
    motor.Move(800, MotorDriver::MOVE_TARGET_ABSOLUTE);
    delay(50);
    
}

void MotorHoming() {
  Serial.println("Press enter to return home");
  WaitForSerial();   // wait for input from user

  // Sets the maximum velocity for moving home
  motor.VelMax(homingVelocityLimit);
  // Set the maximum acceleration for moving home
  motor.AccelMax(homingAccelerationLimit);
  
  motor.Move(-100000, MotorDriver::MOVE_TARGET_ABSOLUTE);
  Serial.println("Homing");

  Serial.println("Moving.. Waiting for HLFB");
  while (!motor.StepsComplete() || motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
      continue;
  }
  
  // Resets the maximum velocity for each move
  motor.VelMax(velocityLimit);
  // Resets the maximum acceleration for each move
  motor.AccelMax(accelerationLimit);
  
  Serial.println("Homing Done");
  delay(100);
  
}

void WaitForSerial(){
  // waits for any input on the serial monitor
  while(Serial.available()){
    Serial.read();
  }
  while(!Serial.available()){}
  Serial.read();
}
//------------------------------------------------------------------------------
