 
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

// *- import edu.wpi.first.networktables.NetworkTableEntry;
// *- import edu.wpi.first.networktables.NetworkTableInstance;
/**
 * Add your docs here.
 */
public class loggingRamseteController extends RamseteController{
    
    private Pose2d m_poseError;
    private Pose2d m_poseTolerance;
    private final double m_b;
    private final double m_zeta;

    // *- NetworkTableEntrycurrentX, currentY, currentRot, desiredX, desiredY, desiredRot, errorX, errorY, errorRot,
                    // desiredVelocity, desiredCurvature, kRamsete, ramseteVX, ramseteVRot;
    
    public loggingRamseteController(double b, double zeta){
        super(b, zeta);
        m_b = b;
        m_zeta = zeta;
        createNetTableInstances();
    }
    public loggingRamseteController(){
        super(2.0, 0.7);
        this.m_b = 2.0;
        this.m_zeta = 0.7;
        createNetTableInstances();
    }
    
    private void createNetTableInstances() {
        // var table = NetworkTableInstance.getDefault().getTable("troubleshooting");

        // currentX = table.getEntry("currentX");
        // currentY = table.getEntry("currentY");
        // currentRot = table.getEntry("currentRot");

        // desiredX = table.getEntry("desiredX");
        // desiredY = table.getEntry("desiredY");
        // desiredRot = table.getEntry("desiredRot");

        // errorX = table.getEntry("errorX");
        // errorY = table.getEntry("errorY");
        // errorRot = table.getEntry("errorRot");

        // desiredVelocity = table.getEntry("desiredVelocity");
        // desiredCurvature = table.getEntry("desiredCurvature");

        // kRamsete = table.getEntry("kRamsete");

        // ramseteVX = table.getEntry("Ramsete vX [m/s]");
        // ramseteVRot = table.getEntry("Ramsete vRot [rad/s]");
    }


    private static double sinc(double x) {
        if (Math.abs(x) < 1e-9) {
          return 1.0 - 1.0 / 6.0 * x * x;
        } else {
          return Math.sin(x) / x;
        }
      }
    

    @Override
    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
            double angularVelocityRefRadiansPerSecond) {
                this.m_poseError = poseRef.relativeTo(currentPose);

                // Aliases for equation readability
                final double eX = m_poseError.getTranslation().getX();
                final double eY = m_poseError.getTranslation().getY();
                final double eTheta = m_poseError.getRotation().getRadians();
                final double vRef = linearVelocityRefMeters;
                final double omegaRef = angularVelocityRefRadiansPerSecond;

                // currentX.setNumber(currentPose.getTranslation().getX());
                // currentY.setNumber(currentPose.getTranslation().getY());
                // currentRot.setNumber(currentPose.getRotation().getDegrees());
        
                // desiredX.setNumber(poseRef.getTranslation().getX());
                // desiredY.setNumber(poseRef.getTranslation().getY());
                // desiredRot.setNumber(poseRef.getRotation().getDegrees());
        
                // errorX.setNumber(m_poseError.getTranslation().getX());
                // errorY.setNumber(m_poseError.getTranslation().getY());
                // errorRot.setNumber(m_poseError.getRotation().getDegrees());
        
                // desiredVelocity.setNumber("desiredVelocity");
                // desiredCurvature.setNumber("desiredCurvature");
            
                double k = 2.0 * m_zeta * Math.sqrt(Math.pow(omegaRef, 2) + m_b * Math.pow(vRef, 2));

                // kRamsete.setNumber(k);

                // ramseteVX.setNumber(vRef * m_poseError.getRotation().getCos() + k * eX);
                // ramseteVRot.setNumber(omegaRef + k * eTheta + m_b * vRef * sinc(eTheta) * eY);

                return new ChassisSpeeds(vRef * m_poseError.getRotation().getCos() + k * eX,
                                         0.0,
                                         omegaRef + k * eTheta + m_b * vRef * sinc(eTheta) * eY);
        
    }
    public ChassisSpeeds calculate(Pose2d currentPose, Trajectory.State desiredState) {
        return calculate(currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond,
            desiredState.velocityMetersPerSecond * desiredState.curvatureRadPerMeter);
      }
    
}