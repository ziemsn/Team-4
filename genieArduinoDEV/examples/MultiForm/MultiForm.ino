// Demo to illustrate the use of some of the new features of this library, and some widgets which have not been illustrated before.
// Also to show how to change forms easily and identify which form you are currently on, and only write data to the widgets found on 
// that form - for efficiency.
// To be used in conjunction with the MultiForm ViSi-Genie Workshop4 demo, found in the /extras folder of this library.
//
// For the basics, please refer to the main genieArduino_Demo and its genieArduino-WS4-Demo Workshop4 application also.

#include <genieArduinoDEV.h>

// Globals
int value1 = 0;
int value2 = 20;
int value3 = 40;
int slider_contrast = 10;
int lastForm = -1;
int j, k, m;

Genie genie;
#define RESETLINE 4  // Arduino pin D4, Change this if you are not using an Arduino Adaptor Shield Version 2 (see code below)

void setup()
{
  // Use a Serial Begin and serial port of your choice in your code and use the
  // genie.Begin function to send it to the Genie library (see this example below)
  // 200K Baud is good for most Arduinos. Galileo should use 115200.
  // Some Arduino variants use Serial1 for the TX/RX pins, as Serial0 is for USB.
  Serial.begin(200000);  // Serial0 @ 200000 (200K) Baud

  // Reset the Display
  // THIS IS IMPORTANT AND CAN PREVENT OUT OF SYNC ISSUES, SLOW SPEED RESPONSE ETC
  // If NOT using a 4D Arduino Adaptor, digitalWrites must be reversed as Display Reset is Active Low, and
  // the 4D Arduino Adaptors invert this signal so must be Active High.
  pinMode(RESETLINE, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  digitalWrite(RESETLINE, 1);  // Reset the Display via D4 (Change this to 0 if wired without Arduino Adaptor Shield)
  delay(100);
  digitalWrite(RESETLINE, 0);  // unReset the Display via D4 (Change this to 1 if wired without Arduino Adaptor Shield)

  while (!genie.Begin(Serial)); // Set up Genie to use Serial port, but also returns if the Display has responded and is online
  // Program will not progress past this point until the display has responded as being online

  // When the display has responded above, do the following once its online 
  // (illustrates how this function can be used, however it is effectively done in the Begin statement already)
  if (genie.IsOnline()) 
  {
    genie.AttachEventHandler(myGenieEventHandler); // Attach the user function Event Handler for processing events
  }

  //Changes to Form0 - (Already on Form0 but illustrating how to change form)
  genie.SetForm(0);

  // Set the brightness/Contrast of the Display - (Not needed but illustrates how)
  // Most Displays use 0-15 for Brightness Control, where 0 = Display OFF, though to 15 = Max Brightness ON.
  // Some displays are more basic, 1 (or higher) = Display ON, 0 = Display OFF.
  genie.WriteContrast(slider_contrast); // initialised above in globals to 10, about 2/3 Max Brightness

  // Set the slider to represent the current brightness
  genie.WriteObject(GENIE_OBJ_SLIDER, 0, slider_contrast); 
}

void loop()
{
  static unsigned long waitPeriod1 = millis();
  static unsigned long waitPeriod2 = millis();

  genie.DoEvents(); // This calls the library each loop to process the queued responses from the display

  // Do these actions with 30ms between them
  // ***************************************
  if (millis() >= waitPeriod1)
  {
    simulationData1(); // Used for this demo to generate the values being sent to the display

    // Write to objects on the current form only, saves some processing time
    int currentForm = genie.GetForm();

    switch (currentForm)
    {
      case 0:
        // Write value1 to CoolGauge0
        genie.WriteObject(GENIE_OBJ_COOL_GAUGE, 0, value1);

        // Checks if its written to this form already - we only want to write the string once, so it doesnt flicker
        if (lastForm != currentForm)
        {
          lastForm = currentForm;
          //Write a string to String0 object on Form0, once
          genie.WriteStr(0, "This is Form 0");
        }
        break;

      case 1:
        // Write value2 to LedDigits0
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, value2);
        // Write slider_val to the iLedDigits0 widget
        genie.WriteObject(GENIE_OBJ_ILED_DIGITS, 0, slider_contrast);

        // Checks if its written to this form already - we only want to write the iLabelB once, so it doesnt flicker
        if (lastForm != currentForm)
        {
          lastForm = currentForm;
          //Write a string to iLabelB object 0 on Form1 (Inherent Label B)
          genie.WriteInhLabel(0, "This is Form 1");
        }
        break;

      case 2:
        // Write value3 to iMediaGauge0 (Inherent Media Gauge)
        genie.WriteObject(GENIE_OBJ_IMEDIA_GAUGE, 0, value3);

        // Checks if its written to this form already - we only want to write the iLabelB once, so it doesnt flicker
        if (lastForm != currentForm)
        {
          lastForm = currentForm;
          //Write a string to iLabelB object 1 on Form2 (Inherent Label B)
          genie.WriteInhLabel(1, "This is Form 2");
        }
        break;

        default:
        break;
    }

    waitPeriod1 = millis() + 30; // rerun this code to update in another 30ms time.
  }
  
  // Do these actions with 80ms between them
  // ***************************************
  if (millis() >= waitPeriod2)
  {
    simulationData2(); // Used for this demo to generate the values being sent to the display
    
    // Write to objects on the current form only, saves some processing time
    int currentForm = genie.GetForm();

    switch (currentForm)
    {
      case 2:
        // Write k and m values to to Video0 and Video1 widgets
        genie.WriteObject(GENIE_OBJ_VIDEO, 0, m);
        genie.WriteObject(GENIE_OBJ_VIDEO, 1, k);
        break;
    }

    waitPeriod2 = millis() + 80; // rerun this code to update in another 80ms time.
  }
}

void myGenieEventHandler(void)
{
  genieFrame Event;
  genie.DequeueEvent(&Event); // Remove the next queued event from the buffer, and process it below

  //If the cmd received is from a Reported Event (Events triggered from the Events tab of Workshop4 objects)
  if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
  {
    if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)             // If the Reported Message was from a WinButton
    {
      if (Event.reportObject.index == 0)                              // If WinButton0 (Index = 0) (Data is irrelevant for a Momentary Button)
      {
        genie.SetForm(1);                                             // Go to Form1
      }
    }
    if (Event.reportObject.object == GENIE_OBJ_4DBUTTON)              // If the Reported Message was from a 4DButton (aka Button 01)
    {
      if (Event.reportObject.index == 0)                              // If 4DButton0 (Index = 0) (Data is irrelevant for a Momentary Button)
      {
        genie.SetForm(2);                                             // Go to Form2
      }
    }
    if (Event.reportObject.object == GENIE_OBJ_IMEDIA_BUTTON)         // If the Reported Message was from a iMediaButton
    {
      if (Event.reportObject.index == 0)                              // If iMediaButton0 (Index = 0) (Data is irrelevant for a Momentary Button)
      {
        genie.SetForm(0);                                             // Go to Form0
      }
    }
    if (Event.reportObject.object == GENIE_OBJ_SLIDER)                // If the Reported Message was from a Slider
    {
      if (Event.reportObject.index == 0)                              // If Slider0 (Index = 0)
      {
        slider_contrast = genie.GetEventData(&Event) + 1;             // Get the value of the Slider0 (+1, note: 1-15 range for this demo, as 0 would turn off the display)
        genie.WriteContrast(slider_contrast);                         // Set the Brightness based on the Slider0 value
      }
    }
  }
}

// Code is purely for simulation purposes
void simulationData1(void)
{
  // Write to objects on the current form only, saves some processing time
  int currentForm = genie.GetForm();

  switch (currentForm)
  {
    case 0:
      if (value1 < 100) // Cool Gauge set to be 0-100 range
        value1++;
      else
        value1 = 0;
      break;

    case 1:
      if (value2 < 999) // 0-999 is 0-99.9 on display, as LED Digits example is set up for 3 digits and 1 decimal
        value2++;
      else
        value2 = 0;      
      break;

    case 2:
      if (value3 < 100) // iMedia Gauge set to be 0-100 range
        value3++;
      else
        value3 = 0;     
      break;

    default:
      break;
  }
}

// Code is purely for simulation purposes
void simulationData2(void)
{
  // Write to objects on the current form only, saves some processing time
  int currentForm = genie.GetForm();

  switch (currentForm)
  {
    case 2:
      // Flip Digits Video - 5 frames per digit (0-49 range, 50 frames total)
      if (j < 498)
      {
        k = j % 50;         // 1's digit
        m = (j / 50) * 5;   // 10's digit
        if (k > 45)
          m += k - 45;
        if (k > 49)
          k -= 50;
        j++;
      }
      else
      {
        j = 0;
      }           
      break;

    default:
      break;
  }
}
