/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.geometry.Pose2d;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This command was copied and modified from RamseteCommand on WpiLib.
 * https://github.com/wpilibsuite/allwpilib/blob/master/wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/RamseteCommand.java
 * 
 * A command that uses a RAMSETE controller ({@link RamseteController}) to follow a trajectory
 * {@link Trajectory} with a differential drive.
 *
 * <p>The command handles trajectory-following, and feedforwards internally.  This
 * is intended to be a more-or-less "complete solution" that can be used by teams without a great
 * deal of controls expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to use the onboard
 * PID functionality of a "smart" motor controller) may use the secondary constructor that omits
 * the PID returning only the raw wheel speeds from the RAMSETE controller.
 */
@SuppressWarnings("PMD.TooManyFields")
public class RamseteCommandMerge extends CommandBase {
    private final Timer m_timer = new Timer();
    private final Trajectory m_trajectory;
    private final RamseteController m_follower;
    private final SimpleMotorFeedforward m_feedforward;
    private final DifferentialDriveKinematics m_kinematics;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;
    private final DriveSubsystem m_driveSubsystem;
    private final double m_endEarly;

    private NetworkTableEntry xError, yError, rotError;

    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
     * Performs no PID control and calculates feedforwards; outputs are the raw wheel speeds
     * from the RAMSETE controller and the feedforwards. It will follow the full trajectory
     *
     * @param trajectory            The trajectory to follow.
     */
    public RamseteCommandMerge(Trajectory trajectory, DriveSubsystem drivetrain) {
        m_trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");

        m_driveSubsystem = DriveSubsystem.getInstance();
        m_follower = new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta);
        m_kinematics = Constants.kDriveKinematics;

        m_feedforward = new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter);

        m_endEarly = 1.0d;

        addRequirements(drivetrain);  //m_driveSubsystem

        var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
        xError = table.getEntry("xError");
        yError = table.getEntry("yError");
        rotError = table.getEntry("rotError");




    }

    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
     * Performs no PID control and calculates feedforwards; outputs are the raw wheel speeds
     * from the RAMSETE controller and the feedforwards. It can end early based on parameters.
     *
     * @param trajectory            The trajectory to follow.
     * @param endEarly          This double can allow the Ramsete Command to end the trajectory early. 
     *                          The idea being that vision can calculate a new updated trajectory half way
     *                          through which is more accurate.
     */
    public RamseteCommandMerge(Trajectory trajectory, double endEarly, DriveSubsystem drivetrain) {
        m_trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");

        m_driveSubsystem = drivetrain;  // DriveSubsystem.getInstance();
        m_follower = new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta);
        m_kinematics = Constants.kDriveKinematics;

        m_feedforward = new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter);

        m_endEarly = endEarly;
        
        addRequirements(m_driveSubsystem);
        
        var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
        xError = table.getEntry("xError");
        yError = table.getEntry("yError");
        rotError = table.getEntry("rotError");
    }

    @Override
    public void initialize() {
        System.out.println("RamseteCommandMerge Initializing");
        System.out.println(m_trajectory.toString());

        // -*- System.out.println("Current Pose: " + m_driveSubsystem.getPose().toString());

        m_prevTime = 0;
        var initialState = m_trajectory.sample(0);
        m_prevSpeeds = m_kinematics.toWheelSpeeds(
                new ChassisSpeeds(initialState.velocityMetersPerSecond,
                        0,
                        initialState.curvatureRadPerMeter
                                * initialState.velocityMetersPerSecond));
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;

        Pose2d CURRENTPOSE = m_driveSubsystem.getPose();
        Trajectory.State DESIREDSTATE = m_trajectory.sample(curTime);
        Pose2d poseError = DESIREDSTATE.poseMeters.relativeTo(CURRENTPOSE);
        xError.setNumber(poseError.getTranslation().getX());
        yError.setNumber(poseError.getTranslation().getY());
        rotError.setNumber(poseError.getRotation().getDegrees());


        var targetWheelSpeeds = m_kinematics.toWheelSpeeds(
                m_follower.calculate(CURRENTPOSE, DESIREDSTATE));

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;


        double leftFeedforward =
                    m_feedforward.calculate(leftSpeedSetpoint,
                            (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);
        double rightFeedforward =
                m_feedforward.calculate(rightSpeedSetpoint,
                        (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

        m_driveSubsystem.tankDriveVelocities(leftSpeedSetpoint, rightSpeedSetpoint, leftFeedforward, rightFeedforward);

        m_prevTime = curTime;
        m_prevSpeeds = targetWheelSpeeds;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("RamseteCommandMerge has ended, time of " + m_timer.get());
        m_timer.stop();
        System.out.println(m_driveSubsystem.getPose().toString());
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds()); //  * m_endEarly
    }
}