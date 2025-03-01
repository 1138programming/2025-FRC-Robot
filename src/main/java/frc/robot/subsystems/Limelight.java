// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Limelight extends SubsystemBase {

  /** Creates a new Limelight. */
  private NetworkTable LimelightOneTable;
  // private NetworkTable LimelightTwoTable;

  private double[] botPose;

    private double botPoseX;
    private double botPoseY;
    private double botPoseZ;
    private double roll;
    private double pitch;
    private double yaw;
    private double latency;
    private double numberOfTargetsSeen;
    private double tagSpan;
    private double averageDistance;
    private double averageArea;
    private double tl;



  
  public Limelight() {
    LimelightOneTable = NetworkTableInstance.getDefault().getTable("limelight");
    
    // LimelightTwoTable = NetworkTableInstance.getDefault().getTable("Limelight_Two");
    botPose = LimelightOneTable.getEntry("botpose_wpiblue").getDoubleArray(new double[11]);
    tl = LimelightOneTable.getEntry("Tl").getDouble(0);
    if (botPose.length != 0) {
      botPoseX = botPose[0];
      botPoseY = botPose[1];
      botPoseZ = botPose[2];
      roll = botPose[3];
      pitch = botPose[4];
      yaw = botPose[5];
      latency = botPose[6];
      numberOfTargetsSeen = botPose[7];
      tagSpan = botPose[8];
      averageDistance = botPose[9];
      averageArea = botPose[10];
    }


    

  }

  public double[] getbotpose() {
    return botPose;
  }
  public double getTl() {
    return tl;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumberArray("botpose", botpose);
    botPose = LimelightOneTable.getEntry("botpose_wpiblue").getDoubleArray(new double[11]);
    tl = LimelightOneTable.getEntry("Tl").getDouble(0);

    if (botPose.length != 0) {
      botPoseX = botPose[0];
      botPoseY = botPose[1];
      botPoseZ = botPose[2];
      roll = botPose[3];
      pitch = botPose[4];
      yaw = botPose[5];
      latency = botPose[6];
      numberOfTargetsSeen = botPose[7];
      tagSpan = botPose[8];
      averageDistance = botPose[9];
      averageArea = botPose[10];
    }


  }
}
