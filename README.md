Prior to turning on robot
 - Note robot front, front sticker or where battery cable (currently) is
 - turn wheels so driving motors are pointing left relative to the fron of the robot

Set Wheel Offsets capability to set the encoder offset count based on wheel positions.  2 ways
 - button on dashbaord
 - User button on Roborio

see Constants.java file for various settings to control robot operation.  (field centric settings, speed limits, deadband, motor controller IDs, ...)

Opportunities

 1. Create a dashboard button to set field centric on or off
 2. Setup an autonomous routine to drive the robot in a square with 4 commands (create command similar to drive method but add in seconds for a timed distance)
 3. Adjust the autonomous routine to use the gyro to turn/face the front of the robot for each forward move
 4. Smooth out the reaction of the wheels to controller commands (replace PIDController for the turning motors with the ProfiledPIDController / TrapezoidProfile which was in the original wpilib example swerve code)
 5. Implement the autonomous scripting routines being implemented on the 2021 robot for this swerve drive robot


