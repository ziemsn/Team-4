//Clockwise rotation moves the sled towards the blade
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

int LoadPosition = 250000;

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
  motor.VelMax(3.5*6400);//FIXME, this depends on system
  // Set the maximum acceleration for each move
  motor.AccelMax(80000);

  
}

//when the system restart. reset the motor by calling the following function
void resetMotor() {
  // Enables the motor; homing will begin automatically if "normal" ClearPath automatic homing is enabled
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
  if(motor.PositionRefCommanded() == CutPosition)
    {
      MotorLocationState = MOTOR_IN_CUT_POSITION;
    }else
    {
      MotorLocationState = MOTOR_NOT_IN_CUT_POSITION;
    }
  if(motor.StatusReg().bit.StepsActive)
    {
      MotorRunState = MOTOR_IS_MOVING;
    }else
    {
      MotorRunState = MOTOR_STOPPED;
    }
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


/*
Userseekshome has been moved to HomeSensor.h
*/
//void UserSeeksHome(void){//Check step direction, whether clockwise or anticlockwise is toward blade
//  /* Move towards the blade for homing, then repeat much more slowly to prevent blade deflection. Needs to be adjusted to use limit switch as probe */
//    // Commands a speed of 10000 pulses/sec away from blade for 0.5 seconds
//    Serial.println("Homing . . . Waiting for motor to finish");
//    motor.MoveVelocity(10000);//Move away from blade
//    delay(500);
//    motor.MoveVelocity(-9000);//Move towards blade
//    Serial.println("set vel to -9000, homing");
//    
//    delay(500);
//    detectMotorStates(0);
//    
//    while (MotorRunState != MOTOR_STOPPED || motor.HlfbState() != MotorDriver::HLFB_ASSERTED) 
//    {              
//      
//      detectHomeSensorState();
//      detectMotorStates(0);
//          
//      continue;
//    }
//
//    //Back out after touching the probe, like 3D-printer homing
//    motor.Move(6400); 
//    delay(500);
//    motor.MoveVelocity(-800); //0.125 revolutions per second
//
//    delay(500);
//    detectMotorStates(0);
//
//    while (MotorRunState != MOTOR_STOPPED || motor.HlfbState() != MotorDriver::HLFB_ASSERTED) 
//    {              
//      
//      detectHomeSensorState();
//      detectMotorStates(0);
//          
//      continue;
//    }
//    
//    // Then slows down to 3200 pulses/sec until clamping into the hard stop
//    motor.MoveVelocity(-3200);
//    // Delay so HLFB has time to deassert
////    delay(10);
//    // Waits for HLFB to assert again, meaning the hardstop has been reached
//    while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
//        continue;
//    }
//    // Stop the velocity move now that the hardstop is reached
//    motor.MoveStopAbrupt();
//    motor.PositionRefSet(0);
//    Serial.println("Homed - UserSeeksHome(), triggered by HLFB assertion (hardstop)");
//
//    return;
//}
//    // Move away from the hard stop. Any move away from the hardstop will
//    // conclude the homing sequence.
//    motor.Move(-1000);
//
//    //Repeat, but much slower to prevent blade deflection
//    motor.MoveVelocity(500);
//    // Delay so HLFB has time to deassert
//    delay(10);
//    // Waits for HLFB to assert again, meaning the hardstop has been reached
//    while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
//        continue;
//    }
//    // Stop the velocity move now that the hardstop is reached
//    motor.MoveStopAbrupt();
//    // Zero the motor's reference position after homing to allow for accurate
//    // absolute position moves
//    motor.PositionRefSet(0);
//    // Move away from the hard stop. Any move away from the hardstop will
//    // conclude the homing sequence.
//    motor.Move(-1000);
//    LoadPosition = 1000;


//    // Delay so HLFB has time to deassert
//    Delay_ms(10);
//    // Waits for HLFB to assert, meaning homing is complete
//    Serial.println("Moving away from hardstop... Waiting for motor");
//    while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
//        continue;
//    }
//    Serial.println("Homing Complete. Motor Ready.");
//    
//    return;
//}