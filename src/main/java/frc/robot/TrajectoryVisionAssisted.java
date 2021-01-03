
package frc.robot;

import java.util.List;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.RamseteCommandMerge;
import frc.robot.subsystems.DriveSubsystem;

public class TrajectoryVisionAssisted extends CommandBase {
    int m_visionType;
    Pose2d prevTargetPose = null;
    Trajectory m_startTrajectory;
    boolean hasStartTrajectory;
    boolean isFollowingTrajectory = false;
    double m_startAtTimeElapsed;
    Translation2d m_endTranslation;
    int m_robotCycleSinceGeneration = 0;
    TrajectoryConfig m_trajectoryConfig;

    final int CYCLES_PER_GENERATION = 6; // Put into config or constructor (maybe both) or make it a function of
                                         // velocity
    final double FUTURE_POSE_TIME = 0.022; // Put into config
    final double END_BEFORE_RAMSETE_TIME = 0.1; // Put into config

    RamseteCommandMerge m_ramseteCommand;

    RobotContainer m_robotContainer = new RobotContainer(); // TEMPORARY TO ALLOW ROBOT BUILD
    DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    Pose2d fakeVisionPose;
    double fakeVisionTimePassed;
    boolean fakeVisionUsed;

    NetworkTableEntry networkTableVisionDistance;
    NetworkTableEntry networkTableVisionAngle;

    /**
     * TrajectoryVisionAssisted will construct a ramsete command and then
     * periodically construct Trajectory objects for the ramsete command to give it
     * new paths based on data from vision.
     * 
     * This constructor takes a starting trajectory and will follow it but will
     * constantly check with vision if vision has a better end pose or heading. But
     * it will only start those checks once a given amount of time has passed. Put 0
     * to instantly start the checks.
     * 
     * @param visionType            1: Steamworks Peg (gyro angle) 2: Simulated
     *                              Vision Pose
     * @param startAfterTimeElasped Will only check vision for data after given time
     *                              has elasped.
     * @param startingTrajectory    Starting trajectory to follow.
     */
    public TrajectoryVisionAssisted(int visionType, double startVisionAfterTimeElasped, Trajectory startingTrajectory) {
        m_visionType = visionType;
        m_startAtTimeElapsed = startVisionAfterTimeElasped;
        m_startTrajectory = startingTrajectory;
        hasStartTrajectory = true;

        if (visionType == 2) {
            m_endTranslation = startingTrajectory.sample(startingTrajectory.getTotalTimeSeconds()).poseMeters
                    .getTranslation();
        }

    }

    // TEMPORARY FIRST TESTING
    public TrajectoryVisionAssisted(int visionType, double startAfterTimeElasped, Trajectory startingTrajectory,
            DriveSubsystem drivetrain, RobotContainer robotContainer) {
        m_visionType = visionType;
        m_startAtTimeElapsed = startAfterTimeElasped;
        m_startTrajectory = startingTrajectory;
        hasStartTrajectory = true;

        if (visionType == 2) {
            m_endTranslation = startingTrajectory.sample(startingTrajectory.getTotalTimeSeconds()).poseMeters
                    .getTranslation();
        }

        m_driveSubsystem = drivetrain;
        m_robotContainer = robotContainer;
        fakeVisionTimePassed = 1.2;
        fakeVisionPose = new Pose2d(2.5, -0.35, new Rotation2d(0));
        fakeVisionUsed = false;
    }

    /**
     * TrajectoryVisionAssisted will construct a ramsete command and then
     * periodically construct Trajectory objects for the ramsete command to give it
     * new paths based on data from vision.
     * 
     * This constructor won't create a RamseteCommandMerge until it gets a pose from
     * vision. As soon as it creates and shedules the command it will override the
     * default drive command and take control of the robot
     * 
     * @param visionType List of vision types in first comment block
     */
    public TrajectoryVisionAssisted(int visionType) {
        m_visionType = visionType;
        m_startAtTimeElapsed = 0;
        hasStartTrajectory = false;
    }

    @Override
    public void initialize() {
        m_trajectoryConfig = m_robotContainer.getConfig();

        if (hasStartTrajectory) {
            runNewRamseteCommand(m_startTrajectory);
            isFollowingTrajectory = true;
        }

        switch (m_visionType) {
        case 1:
            var table = NetworkTableInstance.getDefault().getTable("vision");
            networkTableVisionDistance = table.getEntry("distance");
            networkTableVisionAngle = table.getEntry("angle");
        }

    }

    @Override
    public void execute() {
        if (isFollowingTrajectory == false) {
            Pose2d targetPose = getTargetPose(m_visionType);

            // Check if getTargetPose decided vision data not usable
            if (targetPose != null) {
                m_startTrajectory = generateTrajectory(targetPose);
                runNewRamseteCommand(m_startTrajectory);
                isFollowingTrajectory = true;
            }

        } else {
            if (shouldGenerateTrajectory()) {
                Trajectory trajectory = generateTrajectoryUsingVision();
                if (trajectory != null) {
                    m_ramseteCommand.setNewTrajectory(trajectory);
                    m_robotCycleSinceGeneration = 0;
                    isFollowingTrajectory = true;
                }

            }

        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return m_ramseteCommand.getTotalTimeElapsed() > (m_ramseteCommand.getTotalTime() - END_BEFORE_RAMSETE_TIME);
    }

    /**
     * Generate a trajectory using vision data.
     * 
     * If getTargetPose returns null then pass the null along to not follow a new
     * trajectory.
     * 
     * @return
     */
    private Trajectory generateTrajectoryUsingVision() {
        Pose2d targetPose;
        Trajectory.State futureState = m_ramseteCommand.getFutureState(FUTURE_POSE_TIME);
        switch (m_visionType) {

        // Vision on steamworks peg using gyro for final angle assuming angle is zeroed
        // at beggining & robot is 60 degree angle from target
        case 1:
            targetPose = getTargetPose(1);
            if (targetPose == null)
                return null;
            return generateTrajectory(futureState, targetPose);

        // Fake Vision for testing
        case 2:
            targetPose = getTargetPose(2);
            return generateTrajectory(futureState, targetPose);

        // Vision on ball for heading
        // This setup would only work where the ball location is known and ball can't
        // move, uses vision for heading only
        case 3:
            targetPose = getTargetPose(3); // <-- get heading from vision
            if (targetPose == null)
                return null;
            return generateTrajectory(futureState, targetPose);

        }

        return m_startTrajectory;

    }

    private Pose2d getTargetPose(int visionType) {
        Pose2d targetPose = null;
        double x;
        double y;

        switch (visionType) {
        case 1:
            double distanceFromTarget = networkTableVisionDistance.getDouble(0); // get from vision
            double robotAngleToTarget = networkTableVisionAngle.getDouble(0); // get from vision
            final double angleOfSteamworksPeg = -60;

            if (robotAngleToTarget == -99) {
                return null; // Returning null means don't create a trajectory
            } else if (distanceFromTarget <= 0.5 || distanceFromTarget > 10) {
                return null;
            } else if (robotAngleToTarget == 0) {
                targetPose = new Pose2d(distanceFromTarget, 0, Rotation2d.fromDegrees(angleOfSteamworksPeg));
                break;
            }

            // Adjacent = Hypotenuse * cosAngle
            x = distanceFromTarget * Math.cos(Math.abs(robotAngleToTarget));

            // Opposite = Hypotenuse * sinAngle
            y = distanceFromTarget * Math.cos(Math.abs(robotAngleToTarget));

            // Correct the absolute value
            if (robotAngleToTarget < 0) {
                y *= -1;
            }

            targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(angleOfSteamworksPeg));
            break;

        case 2:
            targetPose = fakeVisionPose;
            break;

        case 3:
            double heading = 0; // get from vision
            targetPose = new Pose2d(m_endTranslation, Rotation2d.fromDegrees(heading));
            break;
        }

        if (targetPose.equals(prevTargetPose)) {
            return null;
        }
        prevTargetPose = targetPose;

        return targetPose;
    }

    private boolean shouldGenerateTrajectory() {
        if (m_ramseteCommand.getTotalTimeElapsed() < m_startAtTimeElapsed) {
            return false;
        }

        if (m_visionType == 2) {
            if (fakeVisionUsed) {
                return false;
            }
            fakeVisionUsed = true;
            return true;
        }

        // Code to add a cooldown on swapping to a new trajectory.
        if (m_robotCycleSinceGeneration > CYCLES_PER_GENERATION) {
            return true;
        } else {
            m_robotCycleSinceGeneration++;
            return false;
        }
    }

    // Also check , if vision hasn't change don't gen a new trajectory.

    private Trajectory generateTrajectory(Pose2d targetPose) {
        Pose2d currentPose = m_driveSubsystem.getPose();
        double currentVelocity = m_driveSubsystem.getAverageSpeeds();
        TrajectoryConfig config = m_trajectoryConfig.setStartVelocity(currentVelocity).setEndVelocity(0)
                .setReversed(false);
        return TrajectoryGenerator.generateTrajectory(currentPose, List.of(), targetPose, config);
    }

    private Trajectory generateTrajectory(Trajectory.State futureState, Pose2d targetPose) {
        TrajectoryConfig config = m_trajectoryConfig.setStartVelocity(futureState.velocityMetersPerSecond)
                .setEndVelocity(0).setReversed(false);
        Trajectory trajectory;
        try {
            trajectory = TrajectoryGenerator.generateTrajectory(futureState.poseMeters, List.of(), targetPose, config);
        } catch (Exception e) {
            System.out.println("Trajectory using pose from vision data couldn't be generated");
            return null;
        }

        return trajectory;
    }

    private void runNewRamseteCommand(Trajectory trajectory) {
        m_ramseteCommand = new RamseteCommandMerge(trajectory, m_driveSubsystem);
        m_ramseteCommand.schedule();
    }

    /**
     * This command is very versitile but it needs to have constraints.
     * 
     * Only use vision when x > or < value. Same goes for y and rotation. REPLACE BY
     * NEXT LINE INSTEAD -> vision will only be used after a certain amount of time
     * has elapsed
     * 
     * Vision types: ball heading, outer goal pose, maybe outer goal heading
     * 
     */

}