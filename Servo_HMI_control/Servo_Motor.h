
#include "ClearCore.h"

#define MOTOR_IS_MOVING 3
#define MOTOR_STOPPED 4
#define MOTOR_IN_CUT_POSITION 5
#define MOTOR_NOT_IN_CUT_POSITION 6


// Specifies which motor to move.
// Options are: ConnectorM0, ConnectorM1, ConnectorM2, or ConnectorM3.
#define motor ConnectorM0


int MotorRunState;
int MotorLocationState;

// Boolean variable to track the state of the motor, Stop = 0, Run = 1
bool AxisStopRun = 0;

// Integer variable to track the current step count of the motor's encoder
int AxisCurrentPos = 0;

// Integer variable to store the default move distance for the motor
int AxisMoveDist = 4000;

// Integer variable to store the default move velocity for the motor
int AxisMoveVel = 800;

// Integer variable to store the default move acceleration for the motor
int AxisMoveAccel = 4000;

// Integer variable to store the default dwell time for the motor
int AxisDwell = 800;

// Unsigned long variable to store the timeout for the dwell time
unsigned long AxisDwellTimeout = 0;

// Integer variable to store the torque of the motor
int AxisTorque = 0;

// Integer variable to store the last recorded position of the motor
int AxisCurrentPosLast = 0;

// Integer variable to store the last recorded move distance for the motor
int AxisMoveDistLast = 0;

// Integer variable to store the last recorded move velocity for the motor
int AxisMoveVelLast = 0;

// Integer variable to store the last recorded move acceleration for the motor
int AxisMoveAccelLast = 0;

// Integer variable to store the last recorded dwell time for the motor
int AxisDwellLast = 0;

// Integer variable to store the last recorded torque of the motor
int AxisTorqueLast = 0;

// Integer variable to store the current frame of the animation
int AxisAnimation = 0;

// Integer variable to store the target position for the motor to move to
int AxisMoveTarget = 0;

// Boolean variable to track if the motor has a fault, 0 if no fault, 1 if fault
bool AxisFault = 0;

// Boolean variable to track if the motor is in continuous mode, 1 if continuous, 0 if not
bool AxisContinuous = 0;

// Boolean variable to track if the dwell time has started
bool AxisStartedDwell = 0;


void initMotorParams() {

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
  motor.VelMax(3000);//FIXME, this depends on system
  // Set the maximum acceleration for each move
  motor.AccelMax(2000);
  
}

//when the system restart. reset the motor by calling the following function
void resetMotor() {
    // Enables the motor; homing will begin automatically if "normal" ClearPath automatic homing is enabled
    motor.EnableRequest(true);
    Serial.println("Motor Enabled");
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

//bool isMotorInitialized() {
//  if (MotorState == MOTOR_HOME && MotorState == MOTOR_STOPPED) {
//     return 1;
//  }
//  return 0;
//}


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

void UserSeeksHome(void){//Check step direection, whether clockwise or anticlockwise is toward blade
  /* Move towards the blade for homing, then repeat much more slowly to prevent blade deflection. Needs to be adjusted to use limit switch as probe */
    // Commands a speed of 4000 pulses/sec towards the hardstop for 2 seconds
    Serial.println("Homing . . . Waiting for motor to finish");
    motor.MoveVelocity(4000);
    Delay_ms(2000);
    // Then slows down to 1000 pulses/sec until clamping into the hard stop
    motor.MoveVelocity(1000);
    // Delay so HLFB has time to deassert
    Delay_ms(10);
    // Waits for HLFB to assert again, meaning the hardstop has been reached
    while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        continue;
    }
    // Stop the velocity move now that the hardstop is reached
    motor.MoveStopAbrupt();
    // Move away from the hard stop. Any move away from the hardstop will
    // conclude the homing sequence.
    motor.Move(-1000);

    //Repeat, but much slower to prevent blade deflection
    motor.MoveVelocity(500);
    // Delay so HLFB has time to deassert
    Delay_ms(10);
    // Waits for HLFB to assert again, meaning the hardstop has been reached
    while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        continue;
    }
    // Stop the velocity move now that the hardstop is reached
    motor.MoveStopAbrupt();
    // Zero the motor's reference position after homing to allow for accurate
    // absolute position moves
    motor.PositionRefSet(0);
    // Move away from the hard stop. Any move away from the hardstop will
    // conclude the homing sequence.
    motor.Move(-1000);


    // Delay so HLFB has time to deassert
    Delay_ms(10);
    // Waits for HLFB to assert, meaning homing is complete
    Serial.println("Moving away from hardstop... Waiting for motor");
    while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        continue;
    }
    Serial.println("Homing Complete. Motor Ready.");
    
    return;
}
