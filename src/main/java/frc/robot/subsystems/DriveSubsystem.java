/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


// Imports I added
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class DriveSubsystem extends SubsystemBase {

    // Differential Drive
    private DifferentialDrive m_drive;

    // Motor Controllers specific to this drivebase subsystem
    private WPI_TalonSRX leftMaster, rightMaster;    //, climberTalon;
    private WPI_VictorSPX leftSlave, rightSlave;
    private final int kPIDLoopIdx = 0;
    private final int kTimeoutMs = 1000;

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;
    private SimpleMotorFeedforward motorFeedForward;

    // Pigeon Specific
    private PigeonIMU pigeon;
    private PigeonIMU.FusionStatus fusionStatus;

    // DriveSubsystem is a singleton class as it represents a physical subsystem
    private static DriveSubsystem currentInstance;

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {

        leftMaster = new WPI_TalonSRX(Constants.LEFT_FRONT_MOTOR);
        rightMaster = new WPI_TalonSRX(Constants.RIGHT_FRONT_MOTOR);
        leftSlave = new WPI_VictorSPX(Constants.LEFT_REAR_MOTOR);
        rightSlave = new WPI_VictorSPX(Constants.RIGHT_REAR_MOTOR);
        followMotors();

        // Talon settings and methods for velocity control copied frc2706-2020-FeederSubsystem:
        // https://github.com/FRC2706/2020-2706-Robot-Code/blob/master/src/main/java/frc/robot/subsystems/FeederSubsystem.java
        leftMaster.configFactoryDefault();
        rightMaster.configFactoryDefault();
        
        // Config the feedbacksenor
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);

        // Set the minimum and maximum output when target setpoint isn't in tolarance range
        leftMaster.configNominalOutputForward(Contants.LEFT_DRIVE_NOMIAL, kTimeoutMs);
        leftMaster.configNominalOutputReverse(-Contants.LEFT_DRIVE_NOMIAL, kTimeoutMs);
        leftMaster.configPeakOutputForward(Contants.LEFT_DRIVE_PEAK, kTimeoutMs);
        leftMaster.configPeakOutputReverse(-Constants.LEFT_DRIVE_PEAK, kTimeoutMs);

        rightMaster.configNominalOutputForward(Contants.RIGHT_DRIVE_NOMIAL, kTimeoutMs);
        rightMaster.configNominalOutputReverse(-Contants.RIGHT_DRIVE_NOMIAL, kTimeoutMs);
        rightMaster.configPeakOutputForward(Contants.RIGHT_DRIVE_PEAK, kTimeoutMs);
        rightMaster.configPeakOutputReverse(-Constants.RIGHT_DRIVE_PEAK, kTimeoutMs);

        leftMaster.config_kF(kPIDLoopIdx, Constants.LEFT_DRIVE_PID_F, kTimeoutMs);
        leftMaster.config_kP(kPIDLoopIdx, Constants.LEFT_DRIVE_PID_P, kTimeoutMs);
        leftMaster.config_kI(kPIDLoopIdx, 0, kTimeoutMs);
        leftMaster.config_kD(kPIDLoopIdx, Constants.LEFT_DRIVE_PID_D, kTimeoutMs);
        leftMaster.configAllowableClosedloopError(0, 50, Constants.CAN_TIMEOUT_SHORT);   // FeederSubsystem had this commented out .configAllowableClosedloopError(0, 0, kTimeoutMs);
        leftMaster.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT_SHORT);

        rightMaster.config_kF(kPIDLoopIdx, Constants.RIGHT_DRIVE_PID_F, kTimeoutMs);
        rightMaster.config_kP(kPIDLoopIdx, Constants.RIGHT_DRIVE_PID_P, kTimeoutMs);
        rightMaster.config_kI(kPIDLoopIdx, 0, kTimeoutMs);
        rightMaster.config_kD(kPIDLoopIdx, Constants.RIGHT_DRIVE_PID_D, kTimeoutMs);
        rightMaster.configAllowableClosedloopError(0, 50, Constants.CAN_TIMEOUT_SHORT);
        rightMaster.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT_SHORT);

        // Voltage Compensation with account for a drop is battery voltage as the match goes on.
        leftMaster.enableVoltageCompensation(true);
        rightMaster.enableVoltageCompensation(true);

        m_drive = new DifferentialDrive(leftMaster, rightMaster);

        // All Pigeon setup and methods copied from frc2706-2020-DriveBase2020
        // https://github.com/FRC2706/2020-2706-Robot-Code/blob/master/src/main/java/frc/robot/subsystems/DriveBase2020.java
        pigeon = new PigeonIMU(new WPI_TalonSRX(Constants.PIGEON_ID));
        pigeon.setFusedHeading(0d, Constants.CAN_TIMEOUT_LONG);
        fusionStatus = new PigeonIMU.FusionStatus();

        zeroEncoders();
        m_odometry = new DifferentialDriveOdometry(getHeadingRotation2d());

    }

    /**
     * Make the slave motors follow the master motors.
     */
    public void followMotors() {
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
    }

    /**
     * Initialize the current DriveBase instance
     */
    public static void init() {
        if (currentInstance == null) {
            currentInstance = new DriveSubsystem();
        }
    }

    public static DriveSubsystem getInstance() {
        init();
        return currentInstance;
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        updateOdometry();
    }

    /**
     * Determine current pose (x, y and orientation) using heading and encoder distances.
     */
    public void updateOdometry() {
        
        m_odometry.update(getHeadingRotation2d(), leftMaster.getSelectedSensorPosition(),
                rightMaster.getSelectedSensorPosition());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     * REQUIRED FOR RAMSETE COMMAND
     * 
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Zero the heading of the robot.
     */
    public void zeroHeading() {
        pigeon.setYaw(0, Constants.CAN_TIMEOUT_SHORT);
    }

    /**
     * Returns true if the pigeon has been defined
     * @return True if the pigeon is defined, false otherwise
     */
    public final boolean hasPigeon() {
        return pigeon != null;
    }
    
    /**
     * Tries to get the current angle as reported by the pigeon
     * 
     * @return The current heading (In degrees) or 0 if there is no pigeon.
     */
    public double getHeading() {
        if (!hasPigeon())
            return 0d;
        pigeon.getFusedHeading(fusionStatus);
        return fusionStatus.heading;
    }

    /**
     * Gets the current angle and returns as a Rotation2d
     * 
     * @return Rotation2D is a rotation in a 2d coordinate frame represented by a point on the unit circle (cosine and sine).
     */
    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        // resetEncoders();
        m_odometry.resetPosition(pose, getHeadingRotation2d());
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with velocities.
     *
     * REQUIRED FOR RAMSETE COMMAND
     * 
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVelocities(double leftVelocity, double rightVelocity, double leftFeedforward, double rightFeedforward) {
        leftMaster.set(ControlMode.Velocity, leftVelocity, DemandType.ArbitraryFeedForward,
                leftFeedforward / 12.0);
        rightMaster.set(ControlMode.Velocity, -rightVelocity, DemandType.ArbitraryFeedForward,
                rightFeedforward / 12.0);

        /**
         * The code example is from this post:
         * https://www.chiefdelphi.com/t/falcon-500-closed-loop-velocity/378170/18
         * 
         * Super helpful example code to include the simpleMotorForward
         * falconMotor.set(
         * ControlMode.Velocity,
         * velocityMetresPerSecond * kRotationsPerMetre * 2048 * 0.1,
         * DemandType.ArbitraryFeedForward,
         * simpleMotorForward.calculate(velocityMetresPerSecond) / 12.0
         * );
         * 
         */
    }

    /**
     * Resets the encoders to have a posistion of 0
     */
    public void zeroEncoders() {
        leftMaster.getSensorCollection().setQuadraturePosition(0, Constants.CAN_TIMEOUT_SHORT);
        rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.CAN_TIMEOUT_SHORT);
    }
}
