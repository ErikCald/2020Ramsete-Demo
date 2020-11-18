/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // The driver's controller
    Joystick m_driverController = new Joystick(Constants.kDriverControllerPort);

    // Default Trajectory Config
    TrajectoryConfig defaultConfig;

    //Network Tables
    NetworkTableEntry driveKF, driveKP, driveKD;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        m_robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(() -> m_robotDrive.arcadeDrive(m_driverController.getY(GenericHID.Hand.kLeft),
                        m_driverController.getX(GenericHID.Hand.kRight)), m_robotDrive));




        var table = NetworkTableInstance.getDefault().getTable("drivetrainTuning");
        driveKF = table.getEntry("driveKF");
        driveKP = table.getEntry("driveKP");
        driveKD = table.getEntry("driveKD");
        
        driveKF.setNumber(Constants.RIGHT_DRIVE_PID_F);
        driveKP.setNumber(Constants.RIGHT_DRIVE_PID_P);
        driveKD.setNumber(Constants.RIGHT_DRIVE_PID_D);  

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {

        Command tankDriveVelocity1 = new RunCommand(
            () -> m_robotDrive.tankDriveVelocities(0.5, 0.5),
            m_robotDrive
        );
        new JoystickButton(m_driverController, XboxController.Button.kY.value)
            .whenHeld(tankDriveVelocity1)
            .whenReleased(() -> m_robotDrive.tankDriveVolts(0, 0));

        Command tankDriveVelocity2 = new RunCommand(
        () -> m_robotDrive.tankDriveVelocities(1, 1),
        m_robotDrive);
        new JoystickButton(m_driverController, XboxController.Button.kB.value)
            .whenHeld(tankDriveVelocity2)
            .whenReleased(() -> m_robotDrive.tankDriveVolts(0, 0));

        Command tankDriveVelocity3 = new RunCommand(
        () -> m_robotDrive.tankDriveVelocities(1.5, 1.5),
        m_robotDrive);
        new JoystickButton(m_driverController, XboxController.Button.kA.value)
            .whenHeld(tankDriveVelocity3)
            .whenReleased(() -> m_robotDrive.tankDriveVolts(0, 0));

        Command tankDriveVelocity4 = new RunCommand(
        () -> m_robotDrive.tankDriveVelocities(-0.5, -0.5),
        m_robotDrive);
        new JoystickButton(m_driverController, XboxController.Button.kX.value)
            .whenHeld(tankDriveVelocity4)
            .whenReleased(() -> m_robotDrive.tankDriveVolts(0, 0));

        new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .whenPressed(() -> m_robotDrive.setDrivetrainFPD(driveKF.getDouble(Constants.RIGHT_DRIVE_PID_F), driveKP.getDouble(Constants.RIGHT_DRIVE_PID_P), driveKD.getDouble(Constants.RIGHT_DRIVE_PID_D)));

        new JoystickButton(m_driverController, XboxController.Button.kBack.value)
        .whenPressed(() -> m_robotDrive.setCoastMode());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        defaultConfig = createDefaultConfig();

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                getConfig());

        Trajectory driveForwardTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                List.of(new Pose2d(0, 0, new Rotation2d(0)),

                // End 1 meters straight ahead of where we started, facing forward
                new Pose2d(3.5, 0, new Rotation2d(0))),
                // Pass config
                getConfig(0, 1)); 

        Trajectory turnOnTheSpotTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                List.of(new Pose2d(0, 0, new Rotation2d(0)),

                // End 90 degrees clockwise in the same location
                new Pose2d(4, 0, Rotation2d.fromDegrees(90))),
                // Pass config
                getConfig());
        
        // Trajectory turnOnTheSpot2Trajectory = TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     List.of(),
        //     new Pose2d(0.01, 0, Rotation2d.fromDegrees(40)),
        //     getConfig());

        Trajectory forwardAndRightTraj = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1.5, 0.5)),  //List.of(), // new Translation2d(1.5, 0.5)
            new Pose2d(2.3, 0.5, Rotation2d.fromDegrees(0)),
            getConfig());

        

        m_robotDrive.zeroHeading();
        m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));

        RamseteCommandMerge ramseteCommand = new RamseteCommandMerge(driveForwardTrajectory, m_robotDrive);

        

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.setBrakeMode());       //tankDriveVelocities(0, 0, 0, 0));

    }

    /**
     * Create the default TrajectoryConfig that can be used for every trajectory.
     * 
     * @return TrajectoryConfig of the default config
     */
    private TrajectoryConfig createDefaultConfig() {
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics, 10);

        return new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                        Constants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);
    }
    
    /**
     *  Get Config will either return default config or a modified default config.
     *
     *  @return TrajectoryConfig An object required to use TrajectoryGenerator
     */
    private TrajectoryConfig getConfig() {
        return defaultConfig;
    }

    /**
     *  Get Config will either return default config or a modified default config.
     *
     *  @param initialVelocity The initial velocity in meters per second
     *  @param finalVelocity The final velocity in meters per second
     * 
     *  @return TrajectoryConfig An object required to use TrajectoryGenerator
     */
    private TrajectoryConfig getConfig(double initialVelocity, double finalVelocity) {
        return defaultConfig.setStartVelocity(initialVelocity).setEndVelocity(finalVelocity);
    }

    /**
     *  Get Config will either return default config or a modified default config.
     *
     *  @param driveBackwards will change the config to drive the robot backwards
     * 
     *  @return TrajectoryConfig An object required to use TrajectoryGenerator
     */
    private TrajectoryConfig getConfig(boolean driveBackwards) {
        return defaultConfig.setReversed(driveBackwards);
    }

    /**
     *  Get Config will either return default config or a modified default config.
     *
     *  @param initialVelocity The initial velocity in meters per second
     *  @param finalVelocity The final velocity in meters per second
     *  @param driveBackwards will change the config to drive the robot backwards
     * 
     *  @return TrajectoryConfig An object required to use TrajectoryGenerator
     */
    private TrajectoryConfig getConfig(double initialVelocity, double finalVelocity, boolean driveBackwards) {
        return defaultConfig.setStartVelocity(initialVelocity).setEndVelocity(finalVelocity).setReversed(driveBackwards);
    }

}
