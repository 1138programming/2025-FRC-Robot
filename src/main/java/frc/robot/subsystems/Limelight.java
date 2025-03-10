// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  public Limelight() {
    // LimelightOneTable = NetworkTableInstance.getDefault().getTable("limelight");
    // botpose = LimelightOneTable.getEntry("botpose_wpiblue").getDoubleArray(new
    // double[11]);
  }

  public double[] getbostpose() {
    return botpose;
  }

  public void updateOdom(CommandSwerveDrivetrain drivetrain) {
    boolean doRejectUpdateF = false;
    boolean doRejectUpdateB = false;
    boolean doRejectUpdate = false;

    LimelightHelpers.SetRobotOrientation("limelight", drivetrain.getRotation3d().toRotation2d().getDegrees(),
        0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2F = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight front");
    LimelightHelpers.PoseEstimate mt2B = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight back");
    if (Math.abs(drivetrain.getPigeon2().getAngularVelocityYWorld().getValueAsDouble()) > 360) // if our angular
                                                                                               // velocity is greater
                                                                                               // than 720 degrees per
                                                                                               // second, ignore vision
                                                                                               // updates
    {
      doRejectUpdate = true;
    }
    if (mt2F.tagCount == 0) {
      doRejectUpdateF = true;
    }
    if (mt2B.tagCount == 0) {
      doRejectUpdateB = true;
    }
    if (!doRejectUpdate) {
      if (!doRejectUpdateB && !doRejectUpdateF)
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      drivetrain.addVisionMeasurement(
          mt2F.pose,
          mt2F.timestampSeconds);
      drivetrain.addVisionMeasurement(
          mt2B.pose,
          mt2B.timestampSeconds);
    } else if (!doRejectUpdateB && doRejectUpdateF) {
      drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      drivetrain.addVisionMeasurement(
          mt2B.pose,
          mt2B.timestampSeconds);
    } else if (doRejectUpdateB && !doRejectUpdateF) {
      drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      drivetrain.addVisionMeasurement(
          mt2F.pose,
          mt2F.timestampSeconds);
    }
    poseEstimate = drivetrain.getState().Pose;
    SmartDashboard.putData("Field", field);
    field.setRobotPose(poseEstimate);;
  }

  public Command updateOdomCommand(CommandSwerveDrivetrain drivetrain) {
    return run(() -> updateOdom(drivetrain));
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
