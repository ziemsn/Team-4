=================================================================================================================================================================================

Description:

This ClearCore example demonstrates controlling up to three ClearPath-SD servos with a 4D Systems Gen4 HMI

=================================================================================================================================================================================

Getting Started:

1. Follow the software_setup.txt instructions to set up your servo(s) and install the required Arduino libraries
2. See the included 'Setup Pictures' for device electrical connections. Refer to individual product manuals for specifics on power and communication wiring.
3. Upload the 4D-ClearCore-Demo-No1.4DGenie program to the HMI (refer to 4D Systems documentation for this)
4. Open the 4D-ClearCore-Demo-No1.ino example in Arduino. Upload this code to ClearCore via USB.
	*NOTE* if your ClearPath Input Resolution is not 800 steps/revolution, change the "MotorProgInputRes" in the Arduino example to match

=================================================================================================================================================================================

Interacting with the HMI demo:

This example makes reciprocating Absolute Positon moves between position 0 and a user defined positive distance. Axis 0 can also make Continuous Velocity moves where the servo 
spins at a specified speed until told to do something different. Servos are enabled automatically by the Arduino code upon startup.

Main Screen:
Use the buttons here to Start and Stop the specified move on Axis 0, 1, and 2. These axes correspond to servos connected to M-0, M-1, and M-2 on ClearCore. Any number or 
combination of these motor connectors can be used. Move parameters are specified in the Axis Info screen.

Axis Info Screens:
*Current Position - real time counter of the steps ClearCore has sent to the servo. Max displayable value is 65535
*Torque(%) - feedback of the torque used during the move, as a percent of the servo's peak rated torque. This value is stale when the servo is not in motion
*Move Distance - the absolute target position of move, in the positive direction. Max commandable value is 65535. This value is ignored when making Continuous Velocity moves.
*Move Velocity - maximum velocity of the move
*Move Acceleration - maximum acceleration of the move
*Dwell - time to wait after finishing the move before moving back
*Continuous Mode(!) - use this button to make Continuous Velocity moves instead of Absolute Position moves (only coded for Axis 0 in this demo)
*Fault LED - detects if ClearCore has an Alert set in that axis Alert Register. ClearCore will not command motion if Alerts are present; they must first be cleared.
*Clear Fault - clears the Alert Register

=================================================================================================================================================================================