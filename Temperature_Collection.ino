#include "ClearCore.h"

#define motor ConnectorM0

#define baudRate 9600

int velocityLimit = 5000;
int accelerationLimit = 16000;

double mmPerPulse = 0.00375096184;

int pulsesToMoveMM = 66650;

bool MoveAbsolutePosition(int32_t position);

#define thermistor DI6

int temperature_data[10000][2]
int dataIndex = 0;


void setup() {
  // put your setup code here, to run once:

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
  // put your main code here, to run repeatedly:

  // oscillate position
  MoveAbsolutePosition(pulsesToMoveMM);
  delay(1000);
  MoveAbsolutePosition(0);
  delay(1000);

  // record temperature and time stamp
  temperature_data[dataIndex][0] = analogRead(thermistor);
  temperature_data[dataIndex][1] = millis();
  dataIndex++;

}
