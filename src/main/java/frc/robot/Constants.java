/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {

    /**
     * Drivetrain Constants
     */
    public static int RIGHT_FRONT_MOTOR = 2;    //robotSpecific(2, 2, 3, 2, 2);      // Mergio is id 2, Mergonaut is id 3
    public static int RIGHT_REAR_MOTOR = 4;    //robotSpecific(4, 4, 4, 4, 4);
    public static int LEFT_FRONT_MOTOR = 1;    //robotSpecific(1, 1, 1, 1, 1);  // I assume the id of this years robot is 4, robotSpecific is just an array and returns value of robotID
    public static int LEFT_REAR_MOTOR = 3;    //robotSpecific(3, 3, 2, 3, 3);
    public static int CLIMBER_TALON = -1;      //robotSpecific(10, 10, -1, -1, 16);
    public static int PIGEON_ID = LEFT_REAR_MOTOR;  //robotSpecific(CLIMBER_TALON, CLIMBER_TALON, RIGHT_REAR_MOTOR, LEFT_FRONT_MOTOR, LEFT_REAR_MOTOR, TALON_5_PLYBOY);

    // Timeouts for sending CAN bus commands
    public static final int CAN_TIMEOUT_SHORT = 10;
    public static final int CAN_TIMEOUT_LONG = 100;

    public static final double LEFT_DRIVE_PID_F = 0.0;
    public static final double LEFT_DRIVE_PID_P = 0.0101d;   // Robot Characterization Measured value: 0.0101
    public static final double LEFT_DRIVE_PID_D = 0.0;

    public static final double RIGHT_DRIVE_PID_F = 0.0;
    public static final double RIGHT_DRIVE_PID_P = 0.0101d; // Value of 2020 robot : 0.018d, 
    public static final double RIGHT_DRIVE_PID_D = 0.0;


    public static final double kTrackwidthMeters = 0.583; //CAD Measured TW: 0.69, experimentaly TW: 0.583 
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.1525;  //    6" / 39.37 = 0.1525m
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 1.16;
    public static final double kvVoltSecondsPerMeter = 3.37;
    public static final double kaVoltSecondsSquaredPerMeter = 0.573;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;

    /**
     * IO Constants
     */
    public static final int kDriverControllerPort = 0;

    /** 
    * Auto Constants
     */
    public static final double kMaxSpeedMetersPerSecond = 2;  // Measured with alot of inaccuracy: 3
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2.0;
    public static final double kRamseteZeta = 0.7;
}
