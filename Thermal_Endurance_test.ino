/*
Authors: Nathan Ziems, Oscar Reyes-Sanchez, Joshua Osuala
ENGR 298
University of Indianapolis
R.B. Annis School of Engineering
This program will command the connected servo motor to move the full length available and back to zero forever. 
It collects temperature data every few minutes to determine thermal overhead of the servo system.
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
  Make sure everything is appropriately powered and cable-managed
  */
#include "ClearCore.h"

#define motor ConnectorM0

#define baudRate 9600

int velocityLimit = 3000; //pulses per second
int accelerationLimit = 16000; //pulses per second per second

//Needs to be adjusted according to the system the servo is attached to
//This is the linear movement (mm) per rotational step (pulse)
double mmPerPulse = 0.00375096184; 
//How many pulses does it take to move by a millimeter
int pulsesToMoveMM = 66650;

bool MoveAbsolutePosition(int32_t position);

//Connect thermistor to port A12 (Analog pin 12)
#define thermistor A12
float motorTemp[10000][2];
int dataIndex = 0;

// resistance at 25 degrees C, 100kOhms
#define THERMISTORNOMINAL 10000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000

unsigned long currentTime = millis();
unsigned long previousTime = 0;
const int interval = 3000;  // 5 minutes in milliseconds


void setup() {

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
  Serial.println("Motor Enabled");

  // Waits for HLFB to assert (waits for homing to complete if applicable)
  Serial.println("Waiting for HLFB...");
  while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
      continue;
  }
  Serial.println("Motor Ready");

}

void loop() {

  // oscillate position
  MoveAbsolutePosition(pulsesToMoveMM);
  delay(2000);
  MoveAbsolutePosition(0);
  delay(2000);

  //Record the time and temperature every five minutes
  currentTime = millis();
  if (currentTime - previousTime >= interval) //if more than or equal to five minutes have passed
  {
    previousTime = currentTime;
    /*This is the code to try on friday. Comes from a clearcore example on reading from the analog ports
    
    //////Have this at the very top
    #define adcResolution 12


    //////This part specifically needs to go in setup()
    // Since analog inputs default to analog input mode, there's no need to
    // call Mode().
    // Set the resolution of the ADC to 12-bit
    AdcMgr.AdcResolution(adcResolution);


    // Read the analog input (A-9 through A-12 may be configured as analog
    // inputs).
    int16_t adcResult = ConnectorA12.State();
    // Convert the reading to a voltage.
    float inputVoltage = 10.0 * adcResult / ((1 << adcResolution) - 1); //10 times result, divided by maximum 12-bit value
    // Display the voltage reading to the serial port.
    SerialPort.Send("A-12 input voltage: ");
    SerialPort.Send(inputVoltage);
    SerialPort.SendLine("V.");

    //convert voltage to resistance, then to temperature in Celcius
    float resistance = SERIESRESISTOR / ( (inputVoltage / 24) - 1 )
    motorTemp[dataIndex][0] = 1 / ( log(resistance / THERMISTORNOMINAL) / BCOEFFICIENT + 1 / (TEMPERATURENOMINAL + 273.15)) - 273.15
    motorTemp[dataIndex][1] = millis();
    
    Serial.print("Temperature ", motorTemp[dataIndex][0], " *C at ", motorTemp[dataIndex][1], " milliseconds"); 
    dataIndex++;
    
    */


    // record temperature and time stamp 
    //    motorTemp[dataIndex][0] = analogRead(thermistor);
    Serial.println(motorTemp[dataIndex][0] = 1000);
    motorTemp[dataIndex][1] = millis();

    //convert voltage reading to resistance
    Serial.println(motorTemp[dataIndex][0] = (1023 / motorTemp[dataIndex][0])  - 1);

    //convert resistance to temperature
    float steinhart;
    steinhart = motorTemp[dataIndex][0] / THERMISTORNOMINAL;  // (R/Ro)
    steinhart = log(steinhart);                               // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                                // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);         // + (1/To)
    steinhart = 1.0 / steinhart;                              // Invert, now it's in Kelvin
    motorTemp[dataIndex][0] = steinhart - 273.15;             // convert absolute temp to C
    
    Serial.print("Temperature "); 
    Serial.print(motorTemp[dataIndex][0]);
    Serial.print(" *C ");
    Serial.println(motorTemp[dataIndex][1]);
    dataIndex++;
  }
   


}

bool MoveAbsolutePosition(int position) {

  // Command the move of absolute distance
  motor.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);
  Serial.print("Moving to absolute position: ");
  Serial.println(position);
  //Wait until finished moving
  while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
      continue;
  }
  return true;
  
}
