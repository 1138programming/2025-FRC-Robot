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
  double[] botpose;

  
  public Limelight() {
    LimelightOneTable = NetworkTableInstance.getDefault().getTable("limelight");
    botpose = LimelightOneTable.getEntry("botpose_wpiblue").getDoubleArray(new double[11]);


    

  }

  public double[] getbostpose() {
    return botpose;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumberArray("botpose", botpose);

  }
}
