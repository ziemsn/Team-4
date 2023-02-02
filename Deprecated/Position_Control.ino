//Adapted from Teknic's libraries for the ClearCore and ClearPath software
//Nathan Ziems, Oscar Reyes-Sanchez, Joshua Osuala
//University of Indianapolis
//R.B.Annis School of Engineering
//ENGR 296-298

//Teknic's library for controlling motors
#include "ClearCore.h"

// Specifies which port to use for motor control
// Options are: ConnectorM0, ConnectorM1, ConnectorM2, or ConnectorM3.
#define motor ConnectorM0

// Select the baud rate to match the target serial device
#define baudRate 9600

// Define the velocity and acceleration limits to be used for each move
// Declared in pulses per second
int velocityLimit = 8000; // pulses per sec
int accelerationLimit = 16000; // pulses per sec^2

// Define Distance per Pulses
double inchPerPulse = 0.000147675662992126; // inches per pulse
double mmPerPulse = 0.00375096184; // mm per pulse
//For reference,
// 6771.596482037 pulses to move the carriage one inch
// 266.5982866944 pulses to move the carriage one millimeter

//Declare variables used in position calculation
int pulsesToMoveInches;
int pulsesToMoveMM;
double distanceToMoveInches;
double distanceToMoveMM;

// Declares our user-defined helper function, which is used to command moves to
// the motor. The definition/implementation of this function is at the  bottom
// of the example
bool MoveAbsolutePosition(int32_t position);

//Define an interrupt pin, DI6 is a port on the clearcore connected to a limit switch
#define LimSwitch DI6

// Declare the signature for our interrupt service routine (ISR). The function
// is defined below. It is to be called whenever the LimSwitch changes
void Motorstop();

void setup() {
  // put your setup code here, to run once:
  // Set up the interrupt pin in digital input mode.
  pinMode(LimSwitch, INPUT);

  // Set an ISR to be called when the state of the interrupt pin goes from
  // true to false.
  attachInterrupt(digitalPinToInterrupt(LimSwitch), Motorstop, RISING);

  // Enable digital interrupts.
  interrupts();

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
  //HLFB (Home Limit Feedback)
  Serial.println("Waiting for HLFB...");
  while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
    continue;
  }
  Serial.println("Motor Ready");
}

void loop() {
  // put your main code here, to run repeatedly:

  distanceToMoveMM = PositionInputPrompt();

  pulsesToMoveMM = distanceToMoveMM / mmPerPulse;

  MoveAbsolutePosition(pulsesToMoveMM);
  delay(1000);

}

double PositionInputPrompt() {
  /*
  Optimized input prompting and validation
  */
  bool isValidNumber = true;
  double positionMM = 0;
  
  //Until the user enters a valid value (A number), continue prompting for a valid value
  while (true)
  {
  // Ask for a position in millimeters to move slider to
  Serial.println("Enter desired position in millimeters:");

  //Read User input from serial monitor
  String positionString = Serial.readStringUntil('\n');
  positionString.trim();

  //Checking that the string is actually a number so the rest of the function works properly
  for (int i = 0; i < positionString.length(); i++) {
    if (!isDigit(positionString[i])) {
      isValidNumber = false;
      break;
    }
  }

  if (!isValidNumber) { // positionString is not a valid number
    Serial.println("Invalid input. Please enter a valid number ");
    continue;
  }
  
  //Display user's input to them
  Serial.print("You entered " + positionString + ".");

  positionMM = positionString.toDouble();    // convert input string to integer

  // comparison check to verify input is integer
  if (positionMM < 0 || positionMM > 290) {
    Serial.println("Out of range. Range is 0 mm to 290 mm");
    continue;
  }

  // If the input is valid and within range, exit the loop
  break;
  }
  
  return positionMM;
}

/*------------------------------------------------------------------------------
   MoveAbsolutePosition

      Command step pulses to move the motor's current position to the absolute
      position specified by "position"
      Prints the move status to the USB serial port
      Returns when HLFB asserts (indicating the motor has reached the commanded
      position)

   Parameters:
      int position  - The absolute position, in step pulses, to move to

   Returns: True/False depending on whether the move was successfully triggered.
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
  // To be used for homing
  //motor.PositionRefSet(0);  //resets home position
  //motor.Move(800, MotorDriver::MOVE_TARGET_ABSOLUTE);

}
//------------------------------------------------------------------------------
