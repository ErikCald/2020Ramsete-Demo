/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class DriveSubsystem extends SubsystemBase {

    // Differential Drive
    private DifferentialDrive m_drive;

    // Motor Controllers specific to this drivebase subsystem
    private WPI_TalonSRX leftMaster, rightMaster;    //, climberTalon; // Pigeon may be connected to the climber talon
    private WPI_TalonSRX leftSlave, rightSlave;
    private final int kPIDLoopIdx = 0;
    private final int kTimeoutMs = 1000;

    // Duplicate of SimpleMotorFeedforward meant to be used for testing 
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter);

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;
    private SimpleMotorFeedforward motorFeedForward;

    // Pigeon Specific
    private PigeonIMU pigeon;
    private PigeonIMU.FusionStatus fusionStatus;

    // DriveSubsystem is a singleton class as it represents a physical subsystem
    private static DriveSubsystem currentInstance;

    
    //Network Tables
    NetworkTableEntry m_xEntry, m_yEntry, leftReference, leftMeasurement, rightReference, rightMeasurement,
                leftDelta, rightDelta, leftVoltage, rightVoltage, busVoltage, leftPosistion, rightPosistion;
    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {

        leftMaster = new WPI_TalonSRX(Constants.LEFT_FRONT_MOTOR);
        rightMaster = new WPI_TalonSRX(Constants.RIGHT_FRONT_MOTOR);
        leftSlave = new WPI_TalonSRX(Constants.LEFT_REAR_MOTOR);
        rightSlave = new WPI_TalonSRX(Constants.RIGHT_REAR_MOTOR);
        followMotors();

        // Talon settings and methods for velocity control copied frc2706-2020-FeederSubsystem:
        // https://github.com/FRC2706/2020-2706-Robot-Code/blob/master/src/main/java/frc/robot/subsystems/FeederSubsystem.java
        leftMaster.configFactoryDefault();
        rightMaster.configFactoryDefault();
        leftSlave.configFactoryDefault();
        rightSlave.configFactoryDefault();


        // leftMaster.setInverted(true);
        // leftSlave.setInverted(true);
        rightMaster.setInverted(true);
        rightSlave.setInverted(true);

        // Config the feedbacksenor
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);

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

        // Voltage Compensation will account for the drop in battery voltage as the match goes on.
        // leftMaster.enableVoltageCompensation(true);
        // rightMaster.enableVoltageCompensation(true);

        //m_drive = new DifferentialDrive(leftMaster, rightMaster);

        // All Pigeon setup and methods copied from frc2706-2020-DriveBase2020
        // https://github.com/FRC2706/2020-2706-Robot-Code/blob/master/src/main/java/frc/robot/subsystems/DriveBase2020.java
        pigeon = new PigeonIMU(leftSlave);   //new WPI_TalonSRX(Constants.PIGEON_ID)
        pigeon.setFusedHeading(0d, Constants.CAN_TIMEOUT_LONG);
        fusionStatus = new PigeonIMU.FusionStatus();

        zeroEncoders();
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getCurrentHeading()));


        // // Network Tables
        // NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
        


        var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
        m_xEntry = table.getEntry("X");
        m_yEntry = table.getEntry("Y");

        leftReference = table.getEntry("left_reference");
        leftMeasurement = table.getEntry("left_measurement");
        rightReference = table.getEntry("right_reference");
        rightMeasurement = table.getEntry("right_measurement");

        leftDelta = table.getEntry("left_delta");
        rightDelta = table.getEntry("left_delta");
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

        // System.out.println("get current pose: " + getPose().toString());
        
    }

    /**
     * Determine current pose (x, y and orientation) using heading and encoder distances.
     */
    public void updateOdometry() {
        m_odometry.update(Rotation2d.fromDegrees(getCurrentHeading()), getLeftEncoderPosistion(),
                getRightEncoderPosistion());

        var translation = m_odometry.getPoseMeters().getTranslation();
        m_xEntry.setNumber(translation.getX());
        m_yEntry.setNumber(translation.getY());

       //System.out.printf("------Left/Right Encoder: %d  /  %d,   Left/Right Side in Meters: %.4f  /  %.4f   \n", leftMaster.getSelectedSensorPosition(), rightMaster.getSelectedSensorPosition(), getLeftEncoderPosistion(), getRightEncoderPosistion()); //getPose().toString()
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
     * Get the posistion of the left encoder
     * 
     * @return Encoder posistion value 
     */
    private double getLeftEncoderPosistion() {
        return -(leftMaster.getSelectedSensorPosition() / 4096.0d) * (0.1524*Math.PI);
    }

    /**
     * Get the posistion of the right encoder
     * 
     * @return Encoder posistion value 
     */
    private double getRightEncoderPosistion() {
        return -(rightMaster.getSelectedSensorPosition() / 4096.0d) * (0.1524*Math.PI);
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
    public double getCurrentHeading() {
        if (!hasPigeon())
            return 0d;
        pigeon.getFusedHeading(fusionStatus);
        return fusionStatus.heading;
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        zeroEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getCurrentHeading()));
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        // -*- System.out.println("------------ARCADE DRIVE");
        //^*m_drive.arcadeDrive(fwd, rot);
    }

    public void setControlMode() {
        leftMaster.set(ControlMode.PercentOutput, 0);
        rightMaster.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Controls the left and right sides of the drive directly with velocities.
     *
     * REQUIRED FOR RAMSETE COMMAND
     * 
     * @param leftVelocity the commanded left output
     * @param rightVelocity the commanded right output
     */
    public void tankDriveVelocities(double leftVelocity, double rightVelocity, double leftFeedforward, double rightFeedforward) {
        leftMaster.set(ControlMode.Velocity, metersPerSecondToTalonVelocity(leftVelocity), DemandType.ArbitraryFeedForward,
                leftFeedforward / 12.0);
        rightMaster.set(ControlMode.Velocity, metersPerSecondToTalonVelocity(rightVelocity), DemandType.ArbitraryFeedForward,
                rightFeedforward / 12.0);
        
        double leftMeasuredVelocity = -(talonVelocityToMetersPerSecond(leftMaster.getSelectedSensorVelocity()));
        double rightMeasuredVelocity = -(talonVelocityToMetersPerSecond(rightMaster.getSelectedSensorVelocity()));

        leftMeasurement.setNumber(leftMeasuredVelocity);
        leftReference.setNumber(leftVelocity);
        leftDelta.setNumber(leftVelocity-leftMeasuredVelocity);

        rightMeasurement.setNumber(rightMeasuredVelocity);
        rightReference.setNumber(rightVelocity);
        rightDelta.setNumber(rightVelocity-rightMeasuredVelocity);

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
     * Controls the left and right side of the drivetrain in meters per second
     * 
     * This parameter set is less accurate and calculates feedforward values assuming your acceleration is 0.
     * Purpose is to use this for testing.
     * 
     * @param leftVelocity
     * @param rightVelocity
     */
    public void tankDriveVelocities(double leftVelocity, double rightVelocity) {
        double leftFF = feedforward.calculate(leftVelocity);
        double rightFF = feedforward.calculate(rightVelocity);

        tankDriveVelocities(leftVelocity, rightVelocity, leftFF, rightFF);


    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMaster.set(ControlMode.Current, leftVolts);
        rightMaster.set(ControlMode.Current, rightVolts);
    }

    /**
     * Resets the encoders to have a posistion of 0
     */
    public void zeroEncoders() {
        leftMaster.getSensorCollection().setQuadraturePosition(0, Constants.CAN_TIMEOUT_SHORT);
        rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.CAN_TIMEOUT_SHORT);
    }

    /**
     * Zero the heading of the robot.
     */
    public void zeroHeading() {
        pigeon.setYaw(0, Constants.CAN_TIMEOUT_SHORT);
    }



    /**
     * Converting m/s to talon ticks/100ms
     * 
     * Unit Conversion Method
     */
    private double metersPerSecondToTalonVelocity(double metersPerSecond) {
        double result = metersPerSecond;
        double circumference = Math.PI * (0.1524);    // Pi*Diameter
        double ticksPerMeter = 4096/circumference;    // Ticks per revolution / circumference
        result = result * ticksPerMeter;   // Meters Per Second * ticks per 1 meter
        result = result * 0.1;    // Converting ticks per second to ticks per 100ms

        return result;
    }

    /**
     * Converting talon ticks/100ms to m/s
     * 
     * Unit Conversion Method
     */
    private double talonVelocityToMetersPerSecond(double talonVelocity) {
        double result = talonVelocity;
        result = result*10; //Convert ticks/100ms to ticks/sec

        double circumference = Math.PI * 0.1524;

        double metersPerTick = circumference/4096;
        result = result * metersPerTick;
        return result;
    }


    /**
     * Set FPD values
     */
    public void setDrivetrainFPD(double kF, double kP, double kD) {
        leftMaster.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
        leftMaster.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
        leftMaster.config_kI(kPIDLoopIdx, 0, kTimeoutMs);
        leftMaster.config_kD(kPIDLoopIdx, kD, kTimeoutMs);

        rightMaster.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
        rightMaster.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
        rightMaster.config_kI(kPIDLoopIdx, 0, kTimeoutMs);
        rightMaster.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
    }
}

