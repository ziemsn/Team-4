# Servo-Driven Bolt Band Saw
University of Indianapolis
R.B. Annis School of Engineering - ENGR 298 Team 4
Software Team: Oscar Reyes-Sanchez, Nathan Ziems, Joshua Osuala
Mechanical Team: Jordan Ankney, Joseph Bertrand, Trilok Patel

In collaboration with Cardinal/Fastenal, Team 4 is designing a system that controls the movement of a bolt clamp to algin with a bandsaw for specified cuts. At this stage, we are using a Teknic ClearCore I/O and Motion controller with a ClearPath Servo to move a carriage along a ball screw. The HMI is a 4D systems 4.3 Inch Touchscreen LCD running off of the clearcore. 

The programs contained are written on the foundation of code publicly available from Teknic and 4D Systems.

# How to use
The ClearCore is programmed using the Arduino IDE. In order for it to function properly, you need the clearcore libraries and 4d systems libraries. Also need to install the board in the IDE using the board manager. The HMI is programmed through the Workshop4 IDE provided by 4D systems. The servo is controlled by a clearcore, with load tuning done in the ClearPath MSP provided by Teknic.
