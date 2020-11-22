

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.RamseteCommandMerge;

public class TrajectoryVisionAssisted extends CommandBase {
    int m_visionType;
    Trajectory m_startTrajectory;
    boolean hasStartTrajectory;
    boolean isFollowingTrajectory = false;
    double m_startAtTimeElapsed;
    Translation2d m_endTranslation;
    int m_robotCycleSinceGeneration = 0;
    TrajectoryConfig m_trajectoryConfig;

    final int CYCLES_PER_GENERATION = 6;  // Put into config or constructor (maybe both) or make it a function of velocity
    final double FUTURE_POSE_TIME = 0.022;  // Put into config
    final double END_BEFORE_RAMSETE_TIME = 0.1;  // Put into config

    RamseteCommandMerge m_ramseteCommand;


    /**
     * TrajectoryVisionAssisted will construct a ramsete command and then periodically construct 
     * Trajectory objects to the ramsete command to give it new paths based on data from vision.
     * 
     * This constructor takes a starting trajectory and will follow it but will constantly check 
     * with vision if vision has a better end pose or heading. But it will only start those checks 
     * once a given amount of time has passed. Put 0 to instantly start the checks.
     * 
     * @param visionType 1: UpperGoal-Pose  2: Ball-Heading
     * @param startAfterTimeElasped Will only check vision for data after given time has elasped.
     * @param startingTrajectory Starting trajectory to follow.
     */
    public TrajectoryVisionAssisted(int visionType, double startAfterTimeElasped, Trajectory startingTrajectory) {
        m_visionType = visionType;
        m_startAtTimeElapsed = startAfterTimeElasped;
        m_startTrajectory = startingTrajectory;
        hasStartTrajectory = true;
        
        if (visionType == 2) {
            m_endTranslation = startingTrajectory.sample(startingTrajectory.getTotalTimeSeconds()).poseMeters.getTranslation();
        }

    }

    /**
     * TrajectoryVisionAssisted will construct a ramsete command and then periodically construct 
     * Trajectory objects to the ramsete command to give it new paths based on data from vision.
     * 
     * This constructor won't create a RamseteCommandMerge until it gets a pose from vision. As 
     * soon as it creates and shedules the command it will override the default drive command
     * and take control of the robot
     * 
     * @param visionType 1: UpperGoal-Pose  2: Ball-Heading
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

    }

    @Override
    public void execute() {
        if (isFollowingTrajectory == false) {

            // The logic here could change to better match the senario/vision code/pose math


            Pose2d targetPose = getTargetPose(1);

            // Check if vision data should be used. (if not(-99))
            boolean useVision = false;
            if (useVision) {
                m_startTrajectory = generateTrajectory(targetPose);
                runNewRamseteCommand(m_startTrajectory);
                isFollowingTrajectory = true;
            }

            
        } else {
            if (shouldGenerateTrajectory()) {
                m_ramseteCommand.setNewTrajectory(generateTrajectoryUsingVision());
                m_robotCycleSinceGeneration = 0;
                isFollowingTrajectory = true;
                    
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

    private Trajectory generateTrajectoryUsingVision() {

        Trajectory.State futureState = m_ramseteCommand.getFutureState(FUTURE_POSE_TIME);
        switch (m_visionType) {

            // Vision on upper goal for pose
            case 1:
                Pose2d targetPose = getTargetPose(1); // <-- get pose from vision+posemath
                return generateTrajectory(futureState, targetPose);

            // Vision on ball for heading
            case 2:
                Rotation2d heading = new Rotation2d(); // <-- get heading from vision
                return generateTrajectory(futureState, new Pose2d(m_endTranslation, heading));
        }

        return m_startTrajectory;

    }

    private Pose2d getTargetPose(int visionType) {
        return new Pose2d(); // <-- get pose from vision+posemath
    }
    private boolean shouldGenerateTrajectory() {
        if (m_ramseteCommand.getTotalTimeElapsed() < m_startAtTimeElapsed) {
            return false;
        }
    
        // Also check , if vision hasn't change don't gen a new trajectory.

        if (m_robotCycleSinceGeneration > CYCLES_PER_GENERATION) {
            return true;
        } else {
            m_robotCycleSinceGeneration++;
            return false;
        }
    }

    private Trajectory generateTrajectory(Pose2d targetPose) {
        Pose2d currentPose = m_driveSubsystem.getPose();
        double currentVelocity = m_driveSubsystem.getAverageSpeeds();
        TrajectoryConfig config = m_trajectoryConfig.setStartVelocity(currentVelocity);
        return TrajectoryGenerator.generateTrajectory(currentPose, List.of(), targetPose, config);
    }

    private Trajectory generateTrajectory(Trajectory.State futureState, Pose2d targetPose) {
        TrajectoryConfig config = m_trajectoryConfig.setStartVelocity(futureState.velocityMetersPerSecond);
        return TrajectoryGenerator.generateTrajectory(futureState.poseMeters, List.of(), targetPose, config);
    }

    private void runNewRamseteCommand(Trajectory trajectory) {
        m_ramseteCommand = new RamseteCommandMerge(trajectory, m_driveSubsystem);
        m_ramseteCommand.schedule();
    }



    /**
     * This command is very versitile but it needs to have constraints. 
     * 
     * Only use vision when x > or < value. Same goes for y and rotation. REPLACE BY NEXT LINE
     * INSTEAD -> vision will only be used after a certain amount of time has elapsed
     * 
     * Vision types: ball heading, outer goal pose, maybe outer goal heading
     * 
     */


}