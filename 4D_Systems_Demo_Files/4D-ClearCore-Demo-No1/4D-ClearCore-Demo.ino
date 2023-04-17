/*
   Title: 4D-ClearCore-Demo-No1

   Description: This ClearCore code demonstrates controlling ClearPath-SD servos with a 4D Systems Gen4 HMI
      - 3 Motors are required to be connected to M-0, M-1 and M-2 connectors for this demo to function
      - Please refer to _Readme.txt and software_setup.txt files for more information

   Copyright (c) 2022 This work is free to use, copy and distribute under the terms of
   the standard MIT permissive software license which can be found at https://opensource.org/licenses/MIT
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

#define baudRate 115200               // ClearCore Baud Rate, for 4D Display

// The containers for our motor objects.
char motorConnectorNames[][4] = { "M-0", "M-1", "M-2" };
MotorDriver* motor[4] = {&ConnectorM0, &ConnectorM1, &ConnectorM2, &ConnectorM3};
#define NUM_OF_MOTORS 3               // used to loop through the array of motor objects 

// Defined these to make it clear in the MoveAbsolutePosition function what the parameter takes, rather than simply a number
#define Motor0 0
#define Motor1 1
#define Motor2 2

// Declares our user-defined helper function, which is used to command absolute position moves to a motor.
bool MoveAbsolutePosition(int motornumber, int position);

// Declares our user-defined helper function, which is used to command velocity moves to the motor.
bool MoveAtVelocity(int motornumber, int velocity);

int i, k;
int loops = 0;                        // Used for the Form0 animation sequence
int CurrentForm = -1;                 // The current Form/Page we are on right now
int PreviousForm = -1;                // The Form/Page we came from
int LEDDigitToEdit = -1;              // The LED Digit index which will be edited
int DigitsToEdit = -1;                // The number of digits in the LED Digit being edited

int MotorProgInputRes = 800;          // Motor Programmed Input Resolution (Change this if you change the motor value in MSP)
int SecondsInMinute = 60;             // Seconds in a minute
float ConversionFactor;               // Used to convert RPM into Steps/s

bool AxisStopRun[4] = {0, 0, 0, 0};             // Stop = 0, Run = 1, from Buttons
int AxisCurrentPos[4] = {0, 0, 0, 0};           // The current step count of the motors encoder
int AxisMoveDist[4] = {4000, 4000, 4000, 4000};          // Default SP
int AxisMoveVel[4] = {800, 800, 800, 800};            // Default SP
int AxisMoveAccel[4] = {4000, 4000, 4000, 4000};         // Default SP
int AxisDwell[4] = {800, 800, 800, 800};              // Default SP
unsigned long AxisDwellTimeout[4] = {0, 0, 0, 0}; // Used to compare agains millis() as to when Dwell has completed (first movement)
int AxisTorque[4] = {0, 0, 0, 0};               // Motor Torque
int AxisCurrentPosLast[4] = {0, 0, 0, 0};       // Last value, to prevent writing unnecessarily
int AxisMoveDistLast[4] = {0, 0, 0, 0};         // Last value, to prevent writing unnecessarily
int AxisMoveVelLast[4] = {0, 0, 0, 0};          // Last value, to prevent writing unnecessarily
int AxisMoveAccelLast[4] = {0, 0, 0, 0};        // Last value, to prevent writing unnecessarily
int AxisDwellLast[4] = {0, 0, 0, 0};            // Last value, to prevent writing unnecessarily
int AxisTorqueLast[4] = {0, 0, 0, 0};          // Last value, to prevent writing unnecessarily
int AxisAnimation[4] = {0, 0, 0, 0};            // 0 to 59 Frames, record of where the animation is up to

int AxisMoveTarget[4] = {0, 0, 0, 0};
bool AxisFault[4] = {0, 0, 0, 0};               // 1 if Axis has a Fault, 0 if no Faults
bool AxisContinuous[4] = {0, 0, 0, 0};          // 1 if Axis is in Continuous Mode (keeps running continuously @ velocity), or back/forth to Distance with Dwell etc.
bool AxisStartedDwell[4] = {0, 0, 0, 0};

//Genie Axis Form #s
int Form1AxisAnimationGenieNum[4] = {4, 5, 6, 99};
int Form1StartGenieNum[4] = {0, 2, 4, 99};
int Form1StopGenieNum[4] = {1, 3, 5, 99};

int AxisFormAnimationGenieNum[4] = {7, 8, 9, 99};
int AxisFormCurrentPositionGenieNum[4] = {0, 6, 12, 99};
int AxisFormDistGenieNum[4] = {1, 7, 13, 99};
int AxisFormVelGenieNum[4] = {2, 8, 14, 99};
int AxisFormAccelGenieNum[4] = {3, 9, 15, 99};
int AxisFormDwellGenieNum[4] = {4, 10, 16, 99};
int AxisFormTorqueGenieNum[4] = {5, 11, 17, 99};

int AxisFormFaultLEDGenieNum[4] = {0, 1, 2, 99};

int AxisFormClrFaultGenieNum[4] = {6, 7, 8, 99};
int AxisFormContModeGenieNum[4] = {9, 99, 99, 99};
int AxisFormBackGenieNum[4] = {3, 4, 5, 99};
int AxisFormDistEditGenieNum[4] = {6, 10, 14, 99};
int AxisFormVelEditGenieNum[4] = {7, 11, 15, 99};
int AxisFormAccelEditGenieNum[4] = {8, 12, 16, 99};
int AxisFormDwellEditGenieNum[4] = {9, 13, 17, 99};

char keyvalue[10];                    // Array to hold keyboard character values
int counter = 0;                      // Keyboard number of characters
int temp, sumTemp;                    // Keyboard Temp values

void setup()
{

  // Sets the input clocking rate. This normal rate is ideal for ClearPath step and direction applications.
  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

  // Sets all motor connectors into step and direction mode.
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

  // Converts Revs/minute (RPM) into Steps/second
  ConversionFactor = (float)MotorProgInputRes / (float)SecondsInMinute;

  // Set the motor's HLFB mode to bipolar PWM
  for (int i = 0; i < NUM_OF_MOTORS; i++) 
  {
    // Set the motor's HLFB mode to bipolar PWM
    motor[i]->HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    // Set the HFLB carrier frequency to 482 Hz
    motor[i]->HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    // Sets the maximum velocity for each move
    motor[i]->VelMax(AxisMoveVel[i] * ConversionFactor);
    // Set the maximum acceleration for each move
    motor[i]->AccelMax(AxisMoveAccel[i] * ConversionFactor);
  }

  Serial.begin(115200);    // Debug/Console printing over USB

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

  for (int i = 0; i < NUM_OF_MOTORS; i++) 
  {
    // Enables the motor; homing will begin automatically if "normal" ClearPath automatic homing is enabled
    motor[i]->EnableRequest(true);
    Serial.print("Motor ");
    Serial.print(i);
    Serial.println(" Enabled");
    delay(10);
  }

  genie.SetForm(0); // Change to Form 0
  CurrentForm = 0;

  genie.WriteContrast(15); // Max Brightness (0-15 range)
}

void loop()
{
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
            genie.WriteObject(GENIE_OBJ_VIDEO, 0, i); // 4D Systems
            genie.WriteObject(GENIE_OBJ_VIDEO, 1, i); // and
            genie.WriteObject(GENIE_OBJ_VIDEO, 2, i); // Teknic
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
        for (int i = 0; i < NUM_OF_MOTORS; i++) 
        {
          if (motor[i]->StatusReg().bit.StepsActive) // Running Status
          {
            genie.WriteObject(GENIE_OBJ_VIDEO, Form1AxisAnimationGenieNum[i], AxisAnimation[i]); // Play the Cog animation based on the Axis 1 Animation frame
          }
        }
        break;

      /********************************** Motor Forms ******************************************/
      //If the current form is an Axis form, Calculate the motor index from the CurrentForm
      case 2:
      case 3:
      case 4:
        i = CurrentForm - 2;

        AxisCurrentPos[i] = constrain(motor[i]->PositionRefCommanded(), 0, 65535);

        if (motor[i]->StatusReg().bit.StepsActive) // Running Status
        {
          genie.WriteObject(GENIE_OBJ_VIDEO, AxisFormAnimationGenieNum[i], AxisAnimation[i]); // Play the Cog animation based on the Axis 1 Animation frame
        }

        // Update the LED Digits only if the value has changed
        if (AxisCurrentPos[i] != AxisCurrentPosLast[i])
        {
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, AxisFormCurrentPositionGenieNum[i], AxisCurrentPos[i]); // Update Current Position
          AxisCurrentPosLast[i] = AxisCurrentPos[i];
        }
        if (AxisMoveDist[i] != AxisMoveDistLast[i])
        {
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, AxisFormDistGenieNum[i], AxisMoveDist[i]); // Update Move Distance
          AxisMoveDistLast[i] = AxisMoveDist[i];
        }
        if (AxisMoveVel[i] != AxisMoveVelLast[i])
        {
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, AxisFormVelGenieNum[i], AxisMoveVel[i]); // Update Move Velocity
          AxisMoveVelLast[i] = AxisMoveVel[i];
        }
        if (AxisMoveAccel[i] != AxisMoveAccelLast[i])
        {
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, AxisFormAccelGenieNum[i], AxisMoveAccel[i]); // Update Move Acceleration
          AxisMoveAccelLast[i] = AxisMoveAccel[i];
        }
        if (AxisDwell[i] != AxisDwellLast[i])
        {
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, AxisFormDwellGenieNum[i], AxisDwell[i]); // Update Dwell
          AxisDwellLast[i] = AxisDwell[i];
        }
        if (AxisTorque[i] != AxisTorqueLast[i])
        {
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, AxisFormTorqueGenieNum[i], AxisTorque[i]);  // Update Torque %
          AxisTorqueLast[i] = AxisTorque[i];
        }
        break;

      case 5: // If the current Form is 5 - Edit Parameter
        // Do something here if required
        break;
    }


    /************************************ ANIMATION *******************************************/

    // Increment to the next motion animation frame based on motion direction
    for (int i = 0; i < NUM_OF_MOTORS; i++) 
    {

      //Serial.print("Axis0Driection = "); Serial.println(Axis0Direction);
      if (motor[i]->StatusReg().bit.StepsActive && !motor[i]->StatusReg().bit.MoveDirection)
      {
        if (AxisAnimation[i] < 59)
          AxisAnimation[i]++;
        else
          AxisAnimation[i] = 0;
      }
      else if (motor[i]->StatusReg().bit.StepsActive && motor[i]->StatusReg().bit.MoveDirection)
      {
        if (AxisAnimation[i] > 0)
          AxisAnimation[i]--;
        else
          AxisAnimation[i] = 59;
      }

      /************* MOTOR ***************/

      // Commands motion when the start button is pushed, using either:
      //  - a continuous velocity move, or
      //  - a reciprocating absolute position move between 0 and an end position, with a dwell at each end
      if (AxisStopRun[i]  && !AxisFault[i])
      {
        // define a timeout as soon as the motor has stopped using the user defined dwell time
        if (motor[i]->StepsComplete() && motor[i]->HlfbState() == MotorDriver::HLFB_ASSERTED)
        {
          if (!AxisStartedDwell[i])
          {
            AxisStartedDwell[i] = 1;
            AxisDwellTimeout[i] = millis() + AxisDwell[i];
            Serial.print(motorConnectorNames[i]); Serial.print(" At Position "); Serial.println(AxisMoveTarget[i]);
          }
        }

        // if the move is a continuous velocity move, do nothing with the timeout and command the velocity move (see function definition below)
        if (AxisContinuous[i])
        {
          MoveAtVelocity(i, AxisMoveVel[i] * ConversionFactor);
        }
        // if the move is a position move, only command motion after the dwell timeout time has elapsed
        else {
          if (AxisStartedDwell[i] && millis() >= AxisDwellTimeout[i])
          {
            AxisStartedDwell[i] = 0;
            // if the last target position was 0, the new target position is 'Move Distance'
            if (AxisMoveTarget[i] == 0)
            {
              AxisMoveTarget[i] = AxisMoveDist[i];
              // if the last target position was not 0, the new target position is back to 0
            } else {
              AxisMoveTarget[i] = 0;
            }
            Serial.print(motorConnectorNames[i]); Serial.print(" Starting move "); Serial.println(AxisMoveTarget[i]);
          }
          // command the position move (see function definition below)
          MoveAbsolutePosition(i, AxisMoveTarget[i]); // Move Motor 0 to Distance position
        }
      }


      // Stops the motor once the stop button is hit
      if (!AxisStopRun[i])
      {
        Serial.print(motorConnectorNames[i]); Serial.println(" Stopped");
        motor[i]->MoveStopDecel(AxisMoveAccel[i]);
      }


      // Displays the motor's current measured torque when a measurement is available.
      // This torque value is not updated when the motor is Move Done or In Fault. The last measured value during motion will display during these conditions.
      if (motor[i]->HlfbState() == MotorDriver::HLFB_HAS_MEASUREMENT)
      {
        // Writes the torque measured, as a percent of motor peak torque rating
        AxisTorque[i] = (int(abs(motor[i]->HlfbPercent())));
      }


      // If a new fault is detected, turn on the axis Fault LED
      if (motor[i]->StatusReg().bit.AlertsPresent && !AxisFault[i])
      {
        AxisFault[i] = true;
        Serial.print(motorConnectorNames[i]); Serial.println(" status: 'In Alert'");
        genie.WriteObject(GENIE_OBJ_USER_LED, AxisFormFaultLEDGenieNum[i], 1);
      }
      // If the fault has sucessfully been cleared, turn off the axis Fault LED
      else if (!motor[i]->StatusReg().bit.AlertsPresent && AxisFault[i])
      {
        AxisFault[i] = false;
        genie.WriteObject(GENIE_OBJ_USER_LED, AxisFormFaultLEDGenieNum[i], 0);
      }
    }

    waitPeriod = millis() + 50; // rerun this code in another 50ms time.
  }
}


/*------------------------------------------------------------------------------
  This is the user's event handler. It is called by genieDoEvents()
  when the following conditions are true

    The link is in an IDLE state, and
    There is an event to handle

  The event can be either a REPORT_EVENT frame sent asynchronously
  from the display or a REPORT_OBJ frame sent by the display in
  response to a READ_OBJ (genie.ReadObject) request.
*/
void myGenieEventHandler(void)
{
  genieFrame Event;
  genie.DequeueEvent(&Event); // Remove the next queued event from the buffer, and process it below

  //If the cmd received is from a Reported Event (Events triggered from the Events tab of Workshop4 objects)
  if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
  {
    if (Event.reportObject.object == GENIE_OBJ_4DBUTTON)              // If the Reported Message was from a 4DButton
    {
      for (int i = 0; i < NUM_OF_MOTORS; i++) 
      {

        /***************************** Form 1 4D Buttons **************************/

        if (Event.reportObject.index == Form1StartGenieNum[i])                              // If 4DButton0 (Index = 0) - Axis0 Start
        {
          AxisStopRun[i] = 1;
        }
        else if (Event.reportObject.index == Form1StopGenieNum[i])                         // If 4DButton1 (Index = 1) - Axis0 Stop
        {
          AxisStopRun[i] = 0;
        }

        /***************************** Form 2-4 4D Buttons **************************/
        else if (Event.reportObject.index == AxisFormClrFaultGenieNum[i])                         // If 4DButton6 (Index = 6) - Axis0 Clear Fault
        {
          Serial.print(motorConnectorNames[i]); Serial.println(" Clearing Fault if present");
          if (motor[i]->StatusReg().bit.AlertsPresent)                     // If the ClearLink has an Alert present
          {
            if (motor[i]->StatusReg().bit.MotorInFault)                    // Check if there also is a motor shutdown
            {
              motor[i]->EnableRequest(false);
              delay(10);
              motor[i]->EnableRequest(true);                               // Cycle the enable to clear the motor fault
            }
            motor[i]->ClearAlerts();                                       // Clear the Alert
          }
        }

        else if (Event.reportObject.index == AxisFormContModeGenieNum[i])                         // If 4DButton9 (Index = 9) - Axis0 Continuous Mode
        {
          int temp;
          temp = genie.GetEventData(&Event);                            // Grab the data associated with the button to detemine if its toggled on or off
          if (temp == 1)
          {
            AxisContinuous[i] = 1;                                        // Set Continuous Mode On
          }
          else
          {
            AxisContinuous[i] = 0;                                        // Turn Contunuous Mode Off
          }
        }
      }
    }

    if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)             // If the Reported Message was from a WinButton
    {
      /***************************** Form 1 Winbuttons **************************/

      if (Event.reportObject.index == 0)                              // If Winbutton0 (Index = 0) - Main Screen Axis 1 Info
      {
        genie.SetForm(2);                                             // Change to Form 2
      }

      else if (Event.reportObject.index == 1)                         // If Winbutton1 (Index = 1) - Main Screen Axis 2 Info
      {
        genie.SetForm(3);                                             // Change to Form 3
      }

      else if (Event.reportObject.index == 2)                         // If Winbutton2 (Index = 2) - Main Screen Axis 3 Info
      {
        genie.SetForm(4);                                             // Change to Form 4
      }

      /***************************** Axis Form  Winbuttons **************************/
      for (int i = 0; i < NUM_OF_MOTORS; i++) 
      {
        if (Event.reportObject.index == AxisFormBackGenieNum[i])        // If Winbutton3 (Index = 3) - Axis0 Information Back
        {
          genie.SetForm(1);                                             // Change to Form 1
        }

        else if (Event.reportObject.index == AxisFormDistEditGenieNum[i])                        // If Winbutton6 (Index = 6) - Axis0 Move Distance Edit
        {
          PreviousForm = 2 + i;                                         // Keep a record of the Form number we came from
          LEDDigitToEdit = AxisFormDistGenieNum[i];                     // The LED Digit which will take this edited value
          DigitsToEdit = 5;                                             // The number of Digits (4 or 5)
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 18, 0);               // Clear any previous data from the Edit Parameter screen
          genie.SetForm(5);                                             // Change to Form 5 - Edit Parameter
        }

        else if (Event.reportObject.index == AxisFormVelEditGenieNum[i])                         // If Winbutton7 (Index = 7) - Axis0 Move Velocity Edit
        {
          PreviousForm = 2 + i;                                         // Keep a record of the Form number we came from
          LEDDigitToEdit = AxisFormVelGenieNum[i];                      // The LED Digit which will take this edited value
          DigitsToEdit = 4;                                             // The number of Digits (4 or 5)
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 18, 0);               // Clear any previous data from the Edit Parameter screen
          genie.SetForm(5);                                             // Change to Form 5 - Edit Parameter
        }

        else if (Event.reportObject.index == AxisFormAccelEditGenieNum[i])                       // If Winbutton8 (Index = 8) - Axis0 Move Acceleration Edit
        {
          PreviousForm = 2 + i;                                         // Keep a record of the Form number we came from
          LEDDigitToEdit = AxisFormAccelGenieNum[i];                    // The LED Digit which will take this edited value
          DigitsToEdit = 4;                                             // The number of Digits (4 or 5)
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 18, 0);               // Clear any previous data from the Edit Parameter screen
          genie.SetForm(5);                                             // Change to Form 5 - Edit Parameter
        }

        else if (Event.reportObject.index == AxisFormDwellEditGenieNum[i])                         // If Winbutton9 (Index = 9) - Axis0 Dwell Edit
        {
          PreviousForm = 2 + i;                                         // Keep a record of the Form number we came from
          LEDDigitToEdit = AxisFormDwellGenieNum[i];                    // The LED Digit which will take this edited value
          DigitsToEdit = 4;                                             // The number of Digits (4 or 5)
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 18, 0);               // Clear any previous data from the Edit Parameter screen
          genie.SetForm(5);                                             // Change to Form 5 - Edit Parameter
        }
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


          for (int i = 0; i < NUM_OF_MOTORS; i++) 
          {
            if (LEDDigitToEdit == AxisFormDistGenieNum[i])
            {
              AxisMoveDist[i] = newValue;
            }
            else if (LEDDigitToEdit == AxisFormVelGenieNum[i])
            {
              AxisMoveVel[i] = constrain(newValue, 0, 6000);                // Limit velocity input to 0-6000 range
              motor[i]->VelMax(AxisMoveVel[i] * ConversionFactor);             // Conversion factor applied to sent the motor the RPM (rather than steps/s)
            }
            else if (LEDDigitToEdit == AxisFormAccelGenieNum[i])
            {
              AxisMoveAccel[i] = newValue;
              motor[i]->AccelMax(AxisMoveAccel[i] * ConversionFactor);         // Conversion factor applied to send the motor the RPM/s (rather than steps/s/s)
            }
            else if (LEDDigitToEdit == AxisFormDwellGenieNum[i])
            {
              AxisDwell[i] = newValue;
            }
          }
          genie.SetForm(PreviousForm);            // Return to the Form which triggered the Keyboard
        }
      }
    }
  }
}

/*------------------------------------------------------------------------------
   MoveAbsolutePosition

      Command step pulses to move the motor's current position to the absolute
      position specified by "position"
      Prints the move status to the USB serial port

   Parameters:
      int motornumber - The motor to control, 0, 1, 2 (or 3 if added above)
      int position  - The absolute position, in step pulses, to move to

   Returns: True/False depending on whether the move was successfully triggered.
*/
bool MoveAbsolutePosition(int motornumber, int position)
{
  if (motor[motornumber]->StatusReg().bit.AlertsPresent)
  {
    Serial.print(motorConnectorNames[motornumber]); Serial.println(" status: 'In Alert'. Move Canceled.");
    return false;
  }
  Serial.print(motorConnectorNames[motornumber]); Serial.print(" Moving to absolute position: "); Serial.println(position);

  motor[motornumber]->Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);
  return true;
}
//------------------------------------------------------------------------------

/*------------------------------------------------------------------------------
   MoveAtVelocity

      Command the motor to move at the specified velocity, in pulses/second.
      Prints the move status to the USB serial port

   Parameters:
      int motornumber - The motor to control, 0, 1, 2 (or 3 if added above)
      int velocity  - The velocity, in step pulses/sec, to command

   Returns: True/False depending on whether the move was successfully triggered.
*/
bool MoveAtVelocity(int motornumber, int velocity)
{
  if (motor[motornumber]->StatusReg().bit.AlertsPresent)
  {
    Serial.print(motorConnectorNames[motornumber]); Serial.println(" status: 'In Alert'. Move Canceled.");
    return false;
  }
  Serial.print(motorConnectorNames[motornumber]); Serial.print(" Moving continuously");
  motor[motornumber]->MoveVelocity(velocity);

  return true;
}
//------------------------------------------------------------------------------
