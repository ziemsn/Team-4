/* 
* This is the header file for the servo controls
* In short, this allows for motor setup and use based on user interactions 
* Motor speed can be defined by changing DesiredRPM. This only affects movement towards cutting position. 
* Homing has its own speed. Recommended maximum: 500 RPM
* WARNING: Do not exceed the maximum as high speeds are not tested for accuracy or stability
* Recommended value: 350 RPM or less
*/
#include "ClearCore.h"

#define MOTOR_IS_MOVING 3
#define MOTOR_STOPPED 4
#define MOTOR_IN_CUT_POSITION 5
#define MOTOR_NOT_IN_CUT_POSITION 6


// Specifies which connector the motor is on
//Make sure blue cable from motor connects to the M0 connector on the Clear Core
#define motor ConnectorM0


int MotorRunState;
int MotorLocationState;

int LoadPosition = 250000; //Arbitrary position away from blade, about 200 mm
int DesiredRPM = 350;

const uint8_t motorChannel = 0;
const uint8_t encoderChannel = 0;

void InitMotorParams() {

  // Sets the input clocking rate. This normal rate is ideal for ClearPath
  // step and direction applications.
  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

  // Sets all motor connectors into step and direction mode.
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

  // Set the motor's HLFB mode to bipolar PWM
  motor.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
  // Set the HFLB carrier frequency to 482 Hz
  motor.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
  // Sets the maximum velocity for each move
  motor.VelMax(DesiredRPM*60*6400);//Conversion to Revolutions per second
  // Set the maximum acceleration for each move
  motor.AccelMax(80000);

  
}

//resetMotor cycles power on the motor. If this succesfully clears the fault state, the fault LED will go out
void resetMotor() {

  motor.EnableRequest(false);
  delay(10);
  motor.EnableRequest(true); 
  Serial.println("Motor Reset");
  delay(10);
}


void setMotorStates(int runState, int locationState) {
  MotorRunState = runState;
  MotorLocationState = locationState;
}

int getMotorRunState() {
  return MotorRunState;
}

int getMotorLocationState() {
  return MotorLocationState;
}

void detectMotorStates(int CutPosition)
{
  if(motor.PositionRefCommanded() == CutPosition) //Set state based on motor's reported position
    {
      MotorLocationState = MOTOR_IN_CUT_POSITION;
    }else
    {
      MotorLocationState = MOTOR_NOT_IN_CUT_POSITION;
    }
  if(motor.StatusReg().bit.StepsActive) //Set state based on motor's activity
    {
      MotorRunState = MOTOR_IS_MOVING;
    }else
    {
      MotorRunState = MOTOR_STOPPED;
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
    return true;
}
