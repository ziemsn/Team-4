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
#include "Servo_Motor.h"
#include "Blade_Saw.h"
#include "HomeSensor.h"
#include <genieArduinoDEV.h>
#include <ClearCore.h>


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

// ClearCore Baud Rate, for 4D Display
#define baudRate 115200


int i, k;
int loops = 0;                        // Used for the Form0 animation sequence
int CurrentForm = -1;                 // The current Form/Page we are on right now
int PreviousForm = -1;                // The Form/Page we came from
int LEDDigitToEdit = -1;              // The LED Digit index which will be edited
int DigitsToEdit = -1;                // The number of digits in the LED Digit being edited

int MotorProgInputRes = 6400;          // Motor Programmed Input Resolution (Change this if you change the motor value in MSP)
int SecondsInMinute = 60;             // Seconds in a minute


//Stored Variables
int MoveDist = 0;
int MoveDistLast = 0;
bool fault = false;
int NextForm = 0;

//Genie Form Item indeces
int DistGenieNum = 7;
int StartProcessGenieNum = 12;

int faultLEDGenieNum = 1;
int ClrfaultGenieNum = 8;

int BackGenieNum = 3;
int DistEditGenieNum = 9;
int EditInputDigitNum = 18;
int FinishedCuttingGenieNum = 1;

int StopMotionGenieNum = 4;
int ClampConfirmGenieNum = 0;
int GoMainGenieNum = 3;
int CutSameGenieNum = 2;
int NewCutGenieNum = 10;

char keyvalue[10];                    // Array to hold keyboard character values
int counter = 0;                      // Keyboard number of characters
int temp, sumTemp;                    // Keyboard Temp values


void setup() {

  initMotorParams();

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
  resetMotor();

  genie.SetForm(1); // Change to Form 0 //should add INIT_FORM variable?
  CurrentForm = 1;

  genie.WriteContrast(7); // Max Brightness (0-15 range)

}

void loop() {
  
  static unsigned long waitPeriod = millis();

  //Need to keep monitoring both home sensor and blade states
  detectHomeSensorState();
  detectBladeState();

  genie.DoEvents(); // This calls the library each loop to process the queued responses from the display

  // waitPeriod later is set to millis()+50, running this code every 50ms
  if (millis() >= waitPeriod)
  {
    CurrentForm = genie.GetForm(); // Check what form the display is currently on

    switch (CurrentForm)
    {
      /************************************* FORM 0 *********************************************/

      case 0:         // If the current Form is 0 - Splash Screen
        // Keeping the splash screen open for 5 seconds
        delay(5000);
        genie.SetForm(3); // Change to main screen
        break;

      /************************************* FORM actions *********************************************/

      case 1: //Motor In Motion Screen 
        if(MotorLocationState == LoadPosition)
        {
          if(MotorRunState == MOTOR_STOPPED)
          {
            delay(500);
            genie.SetForm(NextForm); //Switch to Clamp Confirmation screen
          }
        }   
        break;

      case 2:

        break;

      case 3: //main screen
        if (MoveDist != MoveDistLast)
          {
            genie.WriteObject(GENIE_OBJ_LED_DIGITS, DistGenieNum, MoveDist); // Update Move Distance
            MoveDistLast = MoveDist;
          }
          
          break;            
      
      case 4://Bolt Confirmation Screen

        break;

      case 5: // If the current Form is 5 - Edit Parameter
        // Do something here if required
        break;
    }


      /************* MOTOR ***************/
      // command the position move (see function definition below)
      //Need to check Motor state and then start to move
      //Motor State == MOTOR_STOPPED && 
      //MoveAbsolutePosition(MoveDist); // Move Motor 0 to Distance position **********
      


      // If a new fault is detected, turn on the  fault LED
      if (motor.StatusReg().bit.AlertsPresent && !fault)
      {
        fault = true;
        Serial.print(motorConnectorNames[0]); Serial.println(" status: 'In Alert'");
        genie.WriteObject(GENIE_OBJ_USER_LED, 1, 1);//Set user led 1, to value 1(On)
      }
      // If the fault has sucessfully been cleared, turn off the  fault LED
      else if (!motor.StatusReg().bit.AlertsPresent && fault)
      {
        fault = false;
        genie.WriteObject(GENIE_OBJ_USER_LED, 1, 0);
      }

    waitPeriod = millis() + 50; // rerun this code in another 50ms time.
  }
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

      /***************************** Form 4D Buttons **************************/
         
    
    }

    if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)             // If the Reported Message was from a WinButton
    {
      /*
      //check for back button press
        if (Event.reportObject.index == BackGenieNum)        // If Winbutton3 (Index = 3) - 0 Information Back
        {
          genie.SetForm(3);                                             // Change to Main Screen
        }
      */

      /***************************** Main Screen Winbuttons **************************/
        
        //Check for clear fault press
        if (Event.reportObject.index == ClrfaultGenieNum)                         //8 is the index of the clear fault button 
        {
          Serial.print(motorConnectorNames[0]); Serial.println(" Clearing fault if present");
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
        
        //check for edit press
        else if (Event.reportObject.index == DistEditGenieNum)                        // If Winbutton6 (Index = 6) - 0 Move Distance Edit
        { 
          if (MotorRunState == MOTOR_STOPPED)
          {
            PreviousForm = 3;                                         // Always eturn to the main screen
            LEDDigitToEdit = DistGenieNum;                     // The LED Digit which will take this edited value
            DigitsToEdit = 6;                                             // The number of Digits (4 or 5)
            genie.WriteObject(GENIE_OBJ_LED_DIGITS, EditInputDigitNum, 0);               // Clear any previous data from the Edit Parameter screen //FIXME
            genie.SetForm(5);                                               // Change to Form 5 - Edit Parameter
          }//else alert user something here
        }

        //Check for Start Process press
        else if (Event.reportObject.index == StartProcessGenieNum)
        {
          if (MotorRunState == MOTOR_STOPPED)
          {
            NextForm = 4; //form to go to after MotorMotion screen
            genie.SetForm(1); //Switch to motor in motion screen
            delay(1000);
            MoveAbsolutePosition(LoadPosition); //Move to Loading position
            
          }
        }

      /***************************** Motor In Motion Screen Winbutton **************************/

      if (Event.reportObject.index == StopMotionGenieNum)                             // If the stop button is pressed
      {
        motor.MoveStopAbrupt(); //Immediately stop the motor
        genie.SetForm(3); //return to main screen
      }

      /***************************** Clamp Confirmation Screen Winbuttons **************************/
      if (Event.reportObject.index == GoMainGenieNum)                             // If 'Go Back' is pressed
      {
        if (MotorRunState == MOTOR_STOPPED)
        {
          genie.SetForm(3); //Return to main screen
        }
      }

      if (Event.reportObject.index == ClampConfirmGenieNum)                             // If Proceed is pressed
      {
        if (MotorRunState == MOTOR_STOPPED)
        {
          if (BladeState == BLADE_UP)
          {
            NextForm = 6; //go to Begin cutting screen after MotorMotion Screen
            genie.SetForm(1); //Motor in Motion Screen
          }
        }

      }
      /***************************** Begin Cutting Screen Winbutton **************************/

      if (Event.reportObject.index == FinishedCuttingGenieNum)                             // If Finished cut is pressed
      {
        if (MotorRunState == MOTOR_STOPPED)
        {
          if (BladeState == BLADE_DOWN)
          {
            genie.SetForm(7);
          }
        }
      }

      /***************************** User Finished Screen Winbutton **************************/

      if (Event.reportObject.index == CutSameGenieNum)                             // If Cut same size is pressed
      {
        if (MotorRunState == MOTOR_STOPPED)
        {
          genie.SetForm(4);
        }
      }

      if (Event.reportObject.index == NewCutGenieNum)                             // If Cut same size is pressed
      {
        if (MotorRunState == MOTOR_STOPPED)
        {
          genie.SetForm(3);
        }
      }

      /***************************** Keypad Screen Winbuttons **************************/

      if (Event.reportObject.index == EditInputDigitNum)                             // If Winbutton18 (Index = 18) - Edit Parameter Cancel
      {
        if (MotorRunState == MOTOR_IS_MOVING)
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
    }

    /***************************** Form 5 Keyboard **************************/

    if (Event.reportObject.object == GENIE_OBJ_KEYBOARD)
    {
      if (MotorRunState == MOTOR_IS_MOVING)
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



            
            MoveDist = newValue; 
            //Need to think of solution for decimals, could add decimal button or force (3 digits, decimal, 2 digits)
            //Need to add conditional for metric or imperial, and conversion from those to steps. MoveDist has units of steps.
            
            
          genie.SetForm(PreviousForm);            // Return to the Form which triggered the Keyboard
      }
    }
  }
    }
}
}