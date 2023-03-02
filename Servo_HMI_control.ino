/*
Authors: Nathan Ziems, Oscar Reyes-Sanchez, Joshua Osuala
ENGR 298
University of Indianapolis
R.B. Annis School of Engineering

This is the most up-to-date version of the program that controls the ClearCore, Clearpath, and 4DSystems display.
Controls display interactions and user I/O
Controls servo motion
Processes user input to move a bolt clamp according to the requested cut length

This program is intended for testing only
Setup:
  Using Arduino IDE, compile and upload this program onto a Teknic Clearcore. 
  ***Make sure to have the Teknic libraries and board installed through their respective managers.
  Physical requirements:
    Clearcore Controller
    Clearpath Motor
    4D Systems Display
    Appropriate power and data cables
    Respective power supplies (24VDC for Clearcore, 75VDC for Clearpath)
  
  ***Before attempting to run this program, make sure the motor has been tuned to the load it will move
  ***This requires a programming cable and the ClearPath MSP software
  
  Connect motor to port labelled "M0" on the Clearcore
  4D systems display to COM1

*/
#include "ClearCore.h"
#include <genieArduinoDEV.h>

Genie genie;

//----------------------------------------------------------------------------------------
// Define the ClearCore COM port connected to the HMI
// This must be done using both the Arduino wrapper "Serialx" and ClearCore library "ConnectorCOMx"
// because both interfaces are used below

//#define SerialPort Serial0    // ClearCore UART Port, connected to 4D Display COM0
#define SerialPort Serial1      // ClearCore UART Port, connected to 4D Display COM1

//#define CcSerialPort ConnectorCOM0 // ClearCore UART Port, connected to 4D Display COM0
#define CcSerialPort ConnectorCOM1   // ClearCore UART Port, connected to 4D Display COM1

//----------------------------------------------------------------------------------------

// Specifies which motor to move.
// Options are: ConnectorM0, ConnectorM1, ConnectorM2, or ConnectorM3.
#define motor ConnectorM0

// The containers for our motor objects.
char motorConnectorNames[1][6] = {"Motor"};

// ClearCore Baud Rate, for 4D Display
#define baudRate 115200

// Define the velocity and acceleration limits to be used for each move
int velocityLimit = 5000; // pulses per sec
int accelerationLimit = 10000; // pulses per sec^2

int i, k;
int loops = 0;                        // Used for the Form0 animation sequence
int CurrentForm = -1;                 // The current Form/Page we are on right now
int PreviousForm = -1;                // The Form/Page we came from
int LEDDigitToEdit = -1;              // The LED Digit index which will be edited
int DigitsToEdit = -1;                // The number of digits in the LED Digit being edited

int MotorProgInputRes = 800;          // Motor Programmed Input Resolution (Change this if you change the motor value in MSP)
int SecondsInMinute = 60;             // Seconds in a minute
float ConversionFactor;               // Used to convert RPM into Steps/s

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

//Genie Axis Form #s
int Form1AxisAnimationGenieNum = 4;
int Form1StartGenieNum = 0;
int Form1StopGenieNum = 1;

int AxisFormAnimationGenieNum = 7;
int AxisFormCurrentPositionGenieNum = 0;
int AxisFormDistGenieNum = 1;
int AxisFormVelGenieNum = 2;
int AxisFormAccelGenieNum = 3;
int AxisFormDwellGenieNum = 4;
int AxisFormTorqueGenieNum = 5;

int AxisFormFaultLEDGenieNum = 0;

int AxisFormClrFaultGenieNum = 6;
int AxisFormContModeGenieNum = 7;
int AxisFormBackGenieNum = 3;
int AxisFormDistEditGenieNum = 4;
int AxisFormVelEditGenieNum = 5;
int AxisFormAccelEditGenieNum = 6;
int AxisFormDwellEditGenieNum = 7;

char keyvalue[10];                    // Array to hold keyboard character values
int counter = 0;                      // Keyboard number of characters
int temp, sumTemp;                    // Keyboard Temp values

// Declares our user-defined helper function, which is used to command moves to
// the motor. The definition/implementation of this function is at the  bottom
// of the example
bool MoveAbsolutePosition(int position);


void setup() {

  // Sets the input clocking rate. This normal rate is ideal for ClearPath
  // step and direction applications.
  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

  // Sets all motor connectors into step and direction mode.
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

  // Converts Revs/minute (RPM) into Steps/second
  ConversionFactor = (float)MotorProgInputRes / (float)SecondsInMinute;

  // Set the motor's HLFB mode to bipolar PWM
  motor.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
  // Set the HFLB carrier frequency to 482 Hz
  motor.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
  // Sets the maximum velocity for each move
  motor.VelMax(AxisMoveVel * ConversionFactor);
  // Set the maximum acceleration for each move
  motor.AccelMax(AxisMoveAccel * ConversionFactor);

  // Sets up serial communication and waits up to 5 seconds for a port to open.
  // Serial communication is not required for this example to run.
  Serial.begin(baudRate);

  // Open the HMI serial port and wait until communication has been established
  ConnectorCOM1.RtsMode(SerialBase::LINE_OFF);
  SerialPort.ttl(true);     // Set the Clearcore UART to be TTL
  SerialPort.begin(115200); // Set up Serial1 using the Arduino Serial Class @ 115200 Baud

  while (!SerialPort) {
    continue;
  }

  delay(3000); // Delay to allow Terminal to wake up to capture first debug messages

  while (!genie.Begin(SerialPort));

  if (genie.IsOnline()) // When the display has responded above, do the following once its online
  {
    genie.AttachEventHandler(myGenieEventHandler); // Attach the user function Event Handler for processing events
  }

    // Enables the motor; homing will begin automatically if "normal" ClearPath automatic homing is enabled
    motor.EnableRequest(true);
    Serial.println("Motor Enabled");
    delay(10);

  genie.SetForm(0); // Change to Form 0
  CurrentForm = 0;

  genie.WriteContrast(15); // Max Brightness (0-15 range)

}

void loop() {
  
  static unsigned long waitPeriod = millis();

  genie.DoEvents(); // This calls the library each loop to process the queued responses from the display

  // waitPeriod later is set to millis()+50, running this code every 50ms
  if (millis() >= waitPeriod)
  {
    CurrentForm = genie.GetForm(); // Check what form the display is currently on

    switch (CurrentForm)
    {
      /************************************* FORM 0 *********************************************/

      case 0:         // If the current Form is 0 - Splash Screen
        // Splash Screen Animation
        while (loops < 8) // Play the below, 8 times
        {
          for (i = 0; i < 30; i++) // Play the 30 frames of each of the GIF files
          {
            genie.WriteObject(GENIE_OBJ_VIDEO, 3, i); // ClearCore Demo #1
            delay(25);
          }
          loops++; // Do 8 loops of the Startup Screen, then change Form
        }
        // End Animation

        genie.SetForm(1); // Change to Form 1
        break;

      /************************************* FORM 1 *********************************************/

      case 1: // If the current Form is 1 - Main Screen
          if (motor.StatusReg().bit.StepsActive) // Running Status
          {
            genie.WriteObject(GENIE_OBJ_VIDEO, Form1AxisAnimationGenieNum, AxisAnimation); // Play the Cog animation based on the Axis 1 Animation frame
          }
        break;

      /********************************** Motor Forms ******************************************/
      
      case 4:
      case 3:
      case 2:
        AxisCurrentPos = constrain(motor.PositionRefCommanded(), 0, 65535);

        if (motor.StatusReg().bit.StepsActive) // Running Status
        {
          genie.WriteObject(GENIE_OBJ_VIDEO, AxisFormAnimationGenieNum, AxisAnimation); // Play the Cog animation based on the Axis 1 Animation frame
        }

        // Update the LED Digits only if the value has changed
        if (AxisCurrentPos != AxisCurrentPosLast)
        {
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, AxisFormCurrentPositionGenieNum, AxisCurrentPos); // Update Current Position
          AxisCurrentPosLast = AxisCurrentPos;
        }
        if (AxisMoveDist != AxisMoveDistLast)
        {
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, AxisFormDistGenieNum, AxisMoveDist); // Update Move Distance
          AxisMoveDistLast = AxisMoveDist;
        }
        if (AxisMoveVel != AxisMoveVelLast)
        {
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, AxisFormVelGenieNum, AxisMoveVel); // Update Move Velocity
          AxisMoveVelLast = AxisMoveVel;
        }
        if (AxisMoveAccel != AxisMoveAccelLast)
        {
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, AxisFormAccelGenieNum, AxisMoveAccel); // Update Move Acceleration
          AxisMoveAccelLast = AxisMoveAccel;
        }
        if (AxisDwell != AxisDwellLast)
        {
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, AxisFormDwellGenieNum, AxisDwell); // Update Dwell
          AxisDwellLast = AxisDwell;
        }
        if (AxisTorque != AxisTorqueLast)
        {
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, AxisFormTorqueGenieNum, AxisTorque);  // Update Torque %
          AxisTorqueLast = AxisTorque;
        }
        break;

      case 5: // If the current Form is 5 - Edit Parameter
        // Do something here if required
        break;
    }


    /************************************ ANIMATION *******************************************/

      //Serial.print("Axis0Driection = "); Serial.println(Axis0Direction);
      if (motor.StatusReg().bit.StepsActive && !motor.StatusReg().bit.MoveDirection)
      {
        if (AxisAnimation < 59)
          AxisAnimation++;
        else
          AxisAnimation = 0;
      }
      else if (motor.StatusReg().bit.StepsActive && motor.StatusReg().bit.MoveDirection)
      {
        if (AxisAnimation > 0)
          AxisAnimation--;
        else
          AxisAnimation = 59;
      }

      /************* MOTOR ***************/

      //Dwell is going to be removed
      // Commands motion when the start button is pushed, using 
      //  - a reciprocating absolute position move between 0 and an end position, with a dwell at each end
      if (AxisStopRun  && !AxisFault)
      {
        // define a timeout as soon as the motor has stopped using the user defined dwell time
        if (motor.StepsComplete() && motor.HlfbState() == MotorDriver::HLFB_ASSERTED)
        {
          if (!AxisStartedDwell)
          {
            AxisStartedDwell = 1;
            AxisDwellTimeout = millis() + AxisDwell;
            Serial.print(motorConnectorNames[0]); Serial.print(" At Position "); Serial.println(AxisMoveTarget);
          }
        }

        // if the move is a position move, only command motion after the dwell timeout time has elapsed
//        else {
          if (AxisStartedDwell && millis() >= AxisDwellTimeout)
          {
            AxisStartedDwell = 0;
            // if the last target position was 0, the new target position is 'Move Distance'
            if (AxisMoveTarget == 0)
            {
              AxisMoveTarget = AxisMoveDist;
              // if the last target position was not 0, the new target position is back to 0
            } else {
              AxisMoveTarget = 0;
            }
            Serial.print(motorConnectorNames[0]); Serial.print(" Starting move "); Serial.println(AxisMoveTarget);
          }
          // command the position move (see function definition below)
          MoveAbsolutePosition(AxisMoveTarget); // Move Motor 0 to Distance position**********
//        }
      }


      // Stops the motor once the stop button is hit
      if (!AxisStopRun)
      {
        Serial.print(motorConnectorNames[0]); Serial.println(" Stopped");
        motor.MoveStopDecel(AxisMoveAccel);
      }



      // If a new fault is detected, turn on the axis Fault LED
      if (motor.StatusReg().bit.AlertsPresent && !AxisFault)
      {
        AxisFault = true;
        Serial.print(motorConnectorNames[0]); Serial.println(" status: 'In Alert'");
        genie.WriteObject(GENIE_OBJ_USER_LED, AxisFormFaultLEDGenieNum, 1);
      }
      // If the fault has sucessfully been cleared, turn off the axis Fault LED
      else if (!motor.StatusReg().bit.AlertsPresent && AxisFault)
      {
        AxisFault = false;
        genie.WriteObject(GENIE_OBJ_USER_LED, AxisFormFaultLEDGenieNum, 0);
      }

    waitPeriod = millis() + 50; // rerun this code in another 50ms time.
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

void UserSeeksHome(void){
  /* Move towards the blade for homing, then repeat much more slowly to prevent blade deflection. Needs to be adjusted to use limit switch as probe */
    // Commands a speed of 4000 pulses/sec towards the hardstop for 2 seconds
    SerialPort.SendLine("Homing . . . Waiting for motor to finish");
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
    // Move away from the hard stop. Any move away from the hardstop will
    // conclude the homing sequence.
    motor.Move(-1000);


    // Delay so HLFB has time to deassert
    Delay_ms(10);
    // Waits for HLFB to assert, meaning homing is complete
    SerialPort.SendLine("Moving away from hardstop... Waiting for motor");
    while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        continue;
    }
    SerialPort.SendLine("Homing Complete. Motor Ready.");
    // Zero the motor's reference position after homing to allow for accurate
    // absolute position moves
    motor.PositionRefSet(0);
    return;
}

void myGenieEventHandler(void)
{
  genieFrame Event;
  genie.DequeueEvent(&Event); // Remove the next queued event from the buffer, and process it below

  //If the cmd received is from a Reported Event (Events triggered from the Events tab of Workshop4 objects)
  if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
  {
    if (Event.reportObject.object == GENIE_OBJ_4DBUTTON)              // If the Reported Message was from a 4DButton
    {

      /***************************** Form 2-4 4D Buttons **************************/
      if (Event.reportObject.index == AxisFormClrFaultGenieNum)                         // If 4DButton6 (Index = 6) - Main Screen Clear Fault
      {
        Serial.print(motorConnectorNames[0]); Serial.println(" Clearing Fault if present");
        if (motor.StatusReg().bit.AlertsPresent)                     // If the ClearLink has an Alert present
        {
          if (motor.StatusReg().bit.MotorInFault)                    // Check if there also is a motor shutdown
          {
            motor.EnableRequest(false);
            delay(10);
            motor.EnableRequest(true);                               // Cycle the enable to clear the motor fault
          }
          motor.ClearAlerts();                                       // Clear the Alert
        }
      }

    
    
    }

    if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)             // If the Reported Message was from a WinButton
    {
      

      /***************************** Axis Form  Winbuttons **************************/
        //check for back button press
        if (Event.reportObject.index == AxisFormBackGenieNum)        // If Winbutton3 (Index = 3) - Axis0 Information Back
        {
          genie.SetForm(3);                                             // Change to Main Screen
        }
        //check for edit press
        else if (Event.reportObject.index == AxisFormDistEditGenieNum)                        // If Winbutton6 (Index = 6) - Axis0 Move Distance Edit
        {
          PreviousForm = 3;                                         // Return to the main screen
          LEDDigitToEdit = AxisFormDistGenieNum;                     // The LED Digit which will take this edited value
          DigitsToEdit = 6;                                             // The number of Digits (4 or 5)
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 18, 0);               // Clear any previous data from the Edit Parameter screen
          genie.SetForm(5);                                             // Change to Form 5 - Edit Parameter
        }

      /***************************** Form 5 Winbuttons **************************/

      if (Event.reportObject.index == 18)                             // If Winbutton18 (Index = 18) - Edit Parameter Cancel
      {
        //Clear any partially entered values from Keyboard, ready for next time
        for (int f = 0; f < 5; f++)
        {
          keyvalue[f] = 0;
        }
        counter = 0;

        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 18, 0);               // Clear any data from the Edit Parameter LED Digit
        genie.SetForm(PreviousForm);                                  // Change to Previous Form
      }
    }

    /***************************** Form 5 Keyboard **************************/

    if (Event.reportObject.object == GENIE_OBJ_KEYBOARD)
    {
      if (Event.reportObject.index == 0)                                  // If keyboard0
      {
        temp = genie.GetEventData(&Event);                                // Store the value of the key pressed
        if (temp >= 48 && temp <= 57 && counter < DigitsToEdit)           // Convert value of key into Decimal, 48 ASCII = 0, 57 ASCII = 9, DigitsToEdit is 4 or 5 digits
        {
          keyvalue[counter] = temp;                                       // Append the decimal value of the key pressed, into an character array
          sumTemp = atoi(keyvalue);                                       // Convert the array into a number
          if (DigitsToEdit == 5)                                          // If we are dealing with a parameter which takes a 5 digit number
          {
            if (sumTemp > 65535)                                          // If the number is > 16 bits (the max a Genie LED Digit can be sent)
            {
              sumTemp = 65535;                                            // Limits the max value that can be typed in on a 5 digit parameter, to be 65535 (16 bit number)
            }
          }

          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 18, sumTemp);           // Prints to LED Digit 18 on Form 5 (max the LED digits can take is 65535)
          counter = counter + 1;                                          // Increment array to next position ready for next key press
        }
        else if (temp == 8)                                               // Check if 'Backspace' Key
        {
          if (counter > 0)
          {
            counter--;                                                    // Decrement the counter to the previous key
            keyvalue[counter] = 0;                                        // Overwrite the position in the array with 0 / null
            genie.WriteObject(GENIE_OBJ_LED_DIGITS, 18, atoi(keyvalue));  // Prints the current array value (as an integer) to LED Digit 18 on Form 5
          }
        }
        else if (temp == 13)                                              // Check if 'Enter' Key
        {
          int newValue = sumTemp;
          //Serial.println(newValue);                                     // for debug

          //Clear values ready for next time
          sumTemp = 0;
          for (int f = 0; f < 5; f++)
          {
            keyvalue[f] = 0;
          }
          counter = 0;



            if (LEDDigitToEdit == AxisFormDistGenieNum)
            {
              AxisMoveDist = newValue; 
              //Need to think of solution for decimals, could add decimal button or force (3 digits, decimal, 2 digits)
              //Need to add conditional for metric or imperial, and conversion from those to steps. AxisMoveDist has units of steps.
            }
            
          genie.SetForm(PreviousForm);            // Return to the Form which triggered the Keyboard
      }
    }
  }
}
