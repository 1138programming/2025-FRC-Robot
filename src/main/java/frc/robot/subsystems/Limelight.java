// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.titaniumtigers4829.TigerHelpers;
import com.titaniumtigers4829.data.PoseEstimate;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {

  /** Creates a new Limelight. */
  // private NetworkTable LimelightOneTable;
  double[] botpose;
  private Pose2d poseEstimate = new Pose2d();
  private final Field2d field = new Field2d();
  private String name;

  public Limelight(String name) {
    this.name = name;
  }

  public double[] getbostpose() {
    return botpose;
  }

  public PoseEstimate updateOdom() {
    PoseEstimate pose = TigerHelpers.getBotPoseEstimate(name);
    if (pose == null) {
      return null;
    }
    if (pose.avgTagDist() > 5) {
      return null;
    }
    return pose;
  }


  public Pose2d getbostpose2d() {
    return poseEstimate;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumberArray("botpose", botpose);
  }
}
