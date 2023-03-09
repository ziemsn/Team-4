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



  genie.SetForm(0); // Change to Form 0
  CurrentForm = 0;

  genie.WriteContrast(15); // Max Brightness (0-15 range)

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

      /************************************* FORM descriptions *********************************************/

      case 1:       
      case 4:
      case 3://main screen
        if (AxisMoveDist != AxisMoveDistLast)
          {
            genie.WriteObject(GENIE_OBJ_LED_DIGITS, AxisFormDistGenieNum, AxisMoveDist); // Update Move Distance
            AxisMoveDistLast = AxisMoveDist;
          }
          
          break;
      case 2:              
        

      case 5: // If the current Form is 5 - Edit Parameter
        // Do something here if required
        break;
    }


      /************* MOTOR ***************/
      // command the position move (see function definition below)
      //Need to check Motor state and then start to move
      //Motor State == MOTOR_STOPPED && 
      MoveAbsolutePosition(AxisMoveTarget); // Move Motor 0 to Distance position **********
      


      // If a new fault is detected, turn on the axis Fault LED
      if (motor.StatusReg().bit.AlertsPresent && !AxisFault)
      {
        AxisFault = true;
        Serial.print(motorConnectorNames[0]); Serial.println(" status: 'In Alert'");
        genie.WriteObject(GENIE_OBJ_USER_LED, 1, 1);//Set user led 1, to value 1(On)
      }
      // If the fault has sucessfully been cleared, turn off the axis Fault LED
      else if (!motor.StatusReg().bit.AlertsPresent && AxisFault)
      {
        AxisFault = false;
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
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 18, 0);               // Clear any previous data from the Edit Parameter screen //FIXME
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



            
            AxisMoveDist = newValue; 
            //Need to think of solution for decimals, could add decimal button or force (3 digits, decimal, 2 digits)
            //Need to add conditional for metric or imperial, and conversion from those to steps. AxisMoveDist has units of steps.
            
            
          genie.SetForm(PreviousForm);            // Return to the Form which triggered the Keyboard
      }
    }
  }
}
}
