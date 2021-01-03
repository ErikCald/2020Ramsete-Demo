
package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class TrajectoryVisionConstraint {
    Pose2d refPose;

    boolean doChecks;
    boolean checkX;
    boolean checkY;
    boolean checkRot;
    
    boolean stopOnFirstSuccess;

    boolean xGreater;
    boolean yGreater;
    boolean rotGreater;

    double xCheck;
    double yCheck;
    double rotCheck;

    /**
     * TrajectoryVisionConstraint can put constraints on when to 
     * use vision during a TrajectoryVisionAssisted command.
     * 
     * This constructor will allow vision to be used the moment
     * it sees something
     */
    public TrajectoryVisionConstraint() {
        doChecks = true;
    }

    /**
     * TrajectoryVisionConstraint can put constraints on when to 
     * use vision during a TrajectoryVisionAssisted command.
     * 
     * This constructor will allow vision to be used the moment
     * the robot 
     * 
     * This constructor will allow vision to be used the moment
     * it sees something
     */
    public TrajectoryVisionConstraint(Pose2d poseArea, boolean doXGreater, boolean doYGreater, boolean doRotGreater, boolean stopOnSuccess) {
        refPose = poseArea;
        doChecks = true;

        double x = refPose.getTranslation().getX();
        double y = refPose.getTranslation().getY();
        Rotation2d rot = refPose.getRotation();
        if (x == 0) checkX = false;
        if (y == 0) checkY = false;
        if (rot == new Rotation2d()) checkRot = false;

        xGreater = doXGreater;
        yGreater = doYGreater;
        rotGreater = doRotGreater;

        stopOnFirstSuccess = stopOnSuccess;
    }


    public boolean useVision(Pose2d currentPose) {
        if (doChecks) {
            Pose2d poseError = refPose.relativeTo(currentPose);  //ref-current
            Translation2d translation = poseError.getTranslation();
            Rotation2d rot = poseError.getRotation();

            // Check if x hasn't past desired ref and return false
            if (checkX) {
                if (xGreater) {
                    if (translation.getX() > 0) return false;       // If error of refx-currentx is positive then refx is > than currentx and so don't use vision
                } else {
                    if (translation.getX() < 0) return false;
                }
            }

            // Check if y hasn't past desired ref and return false
            if (checkY) {
                if (yGreater) {
                    if (translation.getY() > 0) return false;
                } else {
                    if (translation.getY() < 0) return false;
                }
            }

            // Check if rot hasn't past desired ref and return false
            if (checkRot) {
                if (rotGreater) {
                    if (rot.getRadians() > 0) return false;
                } else {
                    if (rot.getRadians() > 0) return false;
                }
            }

            // If none of the constraints are met then we are in the right zone to use vision
            if (stopOnFirstSuccess) doChecks = false;
            return true;
                 
        } else {
            return true;
        }
    }
}
