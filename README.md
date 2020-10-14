# 2020Ramsete-Demo
 Ramsete demo with only a DriveSubsystem, Ramsete command and the necessary files for a TimedRobot project.
 
 
 ## Planning the first test
 
 1. Robot Characterization
 2. Drive forward 1 meter, measured for accuracy
 3. S Curve 1 meter forward and 1 meter right
 4. Example S curve from WPILib Docs
 5. Turning on the spot trajectory
 
### Robot Characterization
All info here comes from WPILib Docs tutorial -> https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/index.html
1. Create a config to match the robot. Make sure that the motor controllers are inverted correctly. Also make sure that you input the robot's wheel diamter in meters! WPILib’s trajectory library assumes units of meters are used universally for distance.
2. Generate project -> deploy project
3. Launch data logger
4. Connect to robot and run all 4 tests. Run a forwards test then a backwards test to take up less space. Click on a test, enable in Auto mode on driverstation, then click disable before the robot hits anything. YOU HAVE TO HIT DISABLE! IT WILL NOT STOP MOVING ON ITS OWN. The robot needs at least 10' to properly do these tests but doesn't need any more than 20'
5. Make sure to save the data. 
6. Launch Data Analyzer
7. Load the data file
8. Click Analyze Data
9. Be sure to record/write/save the values kS, kV, kA and r-squared
10. Change the loop type to Velocity. 
11. Change the Gain Settings Preset to WPILib (2020-)
12. Make sure to change the other presets to match the robot. kV and kA should match the numbers on the left side. (eg. Controller Type: Talon, Encoder EPR: ...). If you have trouble matching these to your robot here is a list of the presets and what they do. https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/analyzing-feedback.html#enter-controller-parameters
13. Click on Calculate Optimal Controller Gains.
14. Record/write/save the values beside kP and kD. In our case, since it is a velocity controller, only the P gain is required.
