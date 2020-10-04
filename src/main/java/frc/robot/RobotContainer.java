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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
    XboxController m_driverController = new XboxController(Constants.kDriverControllerPort);

    // Default Trajectory Config
    TrajectoryConfig defaultConfig;

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

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {

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
                new Pose2d(0, 0, new Rotation2d(0)),

                // End 1 meters straight ahead of where we started, facing forward
                new Pose2d(1, 0, new Rotation2d(0)),
                // Pass config
                getConfig()); 

        Trajectory turnOnTheSpotTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),

                // End 90 degrees clockwise in the same location
                new Pose2d(0.001, 0, Rotation2d.fromDegrees(90)),
                // Pass config
                getConfig());
        
        RamseteCommandMerge ramseteCommand = new RamseteCommandMerge(exampleTrajectory);

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVelocities(0, 0, 0, 0));

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
     *  Get Config will either return default config or change the default config to 
     *  also have an intial and final velocity by giving this method two doubles.
     *
     *  @return TrajectoryConfig with an intial and final velocity of 0
     */
    private TrajectoryConfig getConfig() {
        return defaultConfig;
    }

    /**
     *  Get Config will either return default config or a modified default config to 
     *  also have an intial and final velocity.
     *
     *  @param initialVelocity The initial velocity in meters per second
     *  @param finalVelocity The final velocity in meters per second
     * 
     *  @return TrajectoryConfig with the intial and final velocities from constructor
     */
    private TrajectoryConfig getConfig(double initialVelocity, double finalVelocity) {
        return defaultConfig.setStartVelocity(initialVelocity).setEndVelocity(finalVelocity);
    }


}
