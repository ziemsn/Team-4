/*
Authors: Nathan Ziems, Oscar Reyes-Sanchez, Joshua Osuala
ENGR 298
University of Indianapolis
R.B. Annis School of Engineering
This program will command the connected servo motor to move the full length available and back to zero forever. 
It collects temperature data every few minutes to determine thermal overhead of the servo system.
.txt files are opened and closed every loop to prevent data loss
This program is intended for testing only
Setup:
  Using Arduino IDE, compile and upload this program onto a Teknic Clearcore. 
  ***Make sure to have the Teknic libraries and board installed through their respective managers.
  Physical requirements:
    Clearcore Controller
    Clearpath Motor
    Appropriate power and data cables
    Respective power supplies (24VDC for Clearcore, 75VDC for Clearpath)
  
  ***Before attempting to run this program, make sure the motor has been tuned to the load it will move
  ***This requires a programming cable and the ClearPath MSP software
  
  Connect motor to port labelled "M0" on the Clearcore
  Connect thermistor to port A12
  Insert SD card
  Make sure everything is appropriately powered and cable-managed
  */
#include "ClearCore.h"
#include <SD.h>

#define motor ConnectorM0
const int chipSelect = 4; //Select SD card slot
#define baudRate 9600

int velocityLimit = 64000; //pulses per second
int accelerationLimit = 64000; //pulses per second per second
int dwell = 1; //seconds to wait after a move

//Needs to be adjusted according to the system the servo is attached to
//This is the linear movement (mm) per rotational step (pulse)
double mmPerPulse = 0.00375096184; 
//How many pulses does it take to move by a millimeter

bool MoveAbsolutePosition(int32_t position);

//Connect thermistor to port A12 (Analog pin 12)
#define thermistor A12

double motorTemp[10000][2];
int dataIndex = 0;
double inputVoltage;

// Resolution of analog measurement, can be 8, 10, or 12 bits
#define adcResolution 12
// resistance at 25 degrees C, 100kOhms
#define THERMISTORNOMINAL 100000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 98000

unsigned long currentTime = millis();
unsigned long previousTime = 0;
const int interval = 10000;  // Time between data measurements, in milliseconds


void setup() {

  // Since analog inputs default to analog input mode, there's no need to
  // call Mode().
  // Set the resolution of the ADC to 12-bit
  AdcMgr.AdcResolution(adcResolution);

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

    // Enables the motor; homing will begin automatically if enabled
  motor.EnableRequest(true);
  Serial.println("Motor Enabling...");

  // Waits for HLFB to assert (waits for homing to complete if applicable)
  Serial.println("Waiting for motor...");
  while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
      continue;
  }
  Serial.println("Motor Ready");

  // Initialize the SD card.
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed.");
    return;
  }

    // Zero the motor's reference position after homing to allow for accurate
    // absolute position moves
    motor.PositionRefSet(0);

}

void loop() {

  // oscillate position
  MoveAbsolutePosition(500000);
  delay(dwell*1000);
  MoveAbsolutePosition(200000);
  delay(dwell*1000);

  //Record the time and temperature every five minutes
  currentTime = millis();
  if (currentTime - previousTime >= interval) //if more than or equal to five minutes have passed
  {
    previousTime = currentTime;

    // Read the analog input (A-9 through A-12 may be configured as analog
    // inputs).
    inputVoltage = ConnectorA12.AnalogVoltage();

    //convert voltage to resistance
    float resistance = SERIESRESISTOR * ( (2.0 / inputVoltage) - 1 );
    // Serial.println(resistance);//good should be about 130k ohms
    //convert resistance to temperature
    float steinhart;
    steinhart = resistance / THERMISTORNOMINAL;               // (R/Ro
    //Serial.println(steinhart);//good
    steinhart = log(steinhart);                               // ln(R/Ro)
    // Serial.println(steinhart);//good
    steinhart /= BCOEFFICIENT;                                // 1/B * ln(R/Ro) //BCoefficient is wrong
    // Serial.println(steinhart);//technically good, but wrong, and very small
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);         // + (1/To)
    // Serial.println(steinhart);
    steinhart = 1.0 / steinhart;                              // Invert, now it's in Kelvin
    // Serial.println(steinhart);
    motorTemp[dataIndex][0] = steinhart - 273.15;             // convert absolute temp to C
    motorTemp[dataIndex][1] = millis();

    // Open a file for writing.
    File debugFile = SD.open("debug.txt", FILE_WRITE);
      if (!debugFile) {
        Serial.println("Error opening file for writing.");
        return;
      }
    debugFile.print("A-12 input voltage: ");
    debugFile.print(inputVoltage);//good    
    debugFile.println("V. ");
    debugFile.print("Temperature "); 
    debugFile.print(motorTemp[dataIndex][0]); //should be about 20 c
    debugFile.print(" *C, at ");
    debugFile.print(motorTemp[dataIndex][1]); 
    debugFile.println (" milliseconds"); 
    // Close the file.
    debugFile.close();
    Serial.println("Debug written to SD card.");  
    dataIndex++;
    
    // Open a file for writing.
    File dataFile = SD.open("data.txt", FILE_WRITE);
      if (!dataFile) {
        Serial.println("Error opening file for writing.");
        return;
      }
    // Write some data to the file. Formatted as csv with Time, Temperature, Voltage
    dataFile.print(millis() / 1000); 
    dataFile.print(",");
    dataFile.print(steinhart - 273.15);
    dataFile.print(",");
    dataFile.println(inputVoltage);

    // Close the file.
    dataFile.close();
    Serial.println("Data written to SD card.");
    
  }
}

bool MoveAbsolutePosition(int position) {    
  // Command the move of absolute distance
  motor.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);
  
  // Open a file for writing.
    File debugFile = SD.open("debug.txt", FILE_WRITE);
      if (!debugFile) {
        Serial.println("Error opening file for writing.");
        return true;
      }
  debugFile.print(millis() / 1000); 
  debugFile.print("Moving to absolute position: ");
  debugFile.println(position);
  // Waits for HLFB to assert (signaling the move has successfully completed)
  debugFile.println("Moving.. Waiting for motor");
  while (!motor.StepsComplete() || motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        
          if (motor.StatusReg().bit.AlertsPresent)                     // If the ClearLink has an Alert present
          {
            debugFile.println("Faulted. Attempting to clear");
            if (motor.StatusReg().bit.MotorInFault)                    // Check if there also is a motor shutdown
            {
              motor.EnableRequest(false);
              delay(10);
              motor.EnableRequest(true);                               // Cycle the enable to clear the motor fault
            }
            motor.ClearAlerts();  // Clear the Alert
            if (position < 300000)
            {
              motor.PositionRefSet(0);
              
            }else if (position >= 300000)
            {
              motor.PositionRefSet(700000);
              }
          }
        
        continue;
  }
  debugFile.println("Done moving.");
  // Close the file.
  debugFile.close();
  Serial.println("Debug written to SD card.");
  return true;
  
}