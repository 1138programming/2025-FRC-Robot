// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.imageio.plugins.tiff.TIFFTagSet;

import com.ctre.phoenix.Util;
import com.ctre.phoenix6.Utils;
import com.titaniumtigers4829.TigerHelpers;
import com.titaniumtigers4829.data.PoseEstimate;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {

  /** Creates a new Limelight. */
  private NetworkTable LimelightOneTable;
  private double[] botpose;
  private double[] botpose2;
  private Pose2d poseEstimate = new Pose2d();
  private final Field2d field = new Field2d();

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

    // LimelightTwoTable =
    // NetworkTableInstance.getDefault().getTable("Limelight_Two");
    botpose = LimelightOneTable.getEntry("botpose_wpiblue").getDoubleArray(new double[11]);
    tl = LimelightOneTable.getEntry("Tl").getDouble(0);
    if (botpose.length != 0) {
      botPoseX = botpose[0];
      botPoseY = botpose[1];
      botPoseZ = botpose[2];
      roll = botpose[3];
      pitch = botpose[4];
      yaw = botpose[5];
      latency = botpose[6];
      numberOfTargetsSeen = botpose[7];
      tagSpan = botpose[8];
      averageDistance = botpose[9];
      averageArea = botpose[10];
    }
  }

  public double[] getbostpose() {
    return botpose;
  }

  public double getTl() {
    return tl;
  }

  public void updateYaw(CommandSwerveDrivetrain drivetrain) {
    // drivetrain.getPigeon2().setYaw(yaw);
  }

  public void updateOdom(CommandSwerveDrivetrain drivetrain) {
    boolean doRejectUpdateB = false;
    boolean doRejectUpdateF = false;
    boolean doRejectUpdate = false;

    // SmartDashboard.putNumber("rot",
    // drivetrain.getRotation3d().toRotation2d().getDegrees());
    // Set Robot Orientation:
    // LimelightHelpers.SetRobotOrientation("limelight-back",
    // drivetrain.getRotation3d().toRotation2d().getDegrees(),
    // 0, 0, 0, 0, 0);
    // LimelightHelpers.SetRobotOrientation("limelight-front",
    // drivetrain.getRotation3d().toRotation2d().getDegrees(),
    // 0, 0, 0, 0, 0);

    // How much we trust our limelight measurments:
    PoseEstimate mt2F = TigerHelpers.getBotPoseEstimate("limelight-front");
    PoseEstimate mt2B = TigerHelpers.getBotPoseEstimate("limelight-back");

    // Pose Estimates:
    if (!(mt2B == null)) {
      if (Math.abs(drivetrain.getPigeon2().getAngularVelocityYWorld().getValueAsDouble()) > 360) {
        doRejectUpdate = true;
      }
      if (mt2B.tagCount() == 0) {
        doRejectUpdateB = true;
      }
      if (!doRejectUpdate && !doRejectUpdateB) {

        drivetrain.setVisionMeasurementStdDevs(
            VecBuilder.fill(0.01 * Math.pow(mt2B.avgTagDist(), 2), 0.01 *
                Math.pow(mt2B.avgTagDist(), 2), 0));
                mt2B.pose() = new Pose2d(mt2B.pose.getTranslation(),
                drivetrain.getState().Pose.getRotation());
        drivetrain.addVisionMeasurement(
            mt2B.pose(),
            Utils.fpgaToCurrentTime(mt2B.timestampSeconds()));

      }
    }
    if (!(mt2F == null)) {
      if (Math.abs(drivetrain.getPigeon2().getAngularVelocityYWorld().getValueAsDouble()) > 360) {
        doRejectUpdate = true;
      }
      if (mt2F.tagCount() == 0) {
        doRejectUpdateF = true;
      }
      if (!doRejectUpdate && !doRejectUpdateF) {
        drivetrain.setVisionMeasurementStdDevs(
            VecBuilder.fill(0.01 * Math.pow(mt2F.avgTagDist(), 2), 0.01 *
                Math.pow(mt2F.avgTagDist(), 2), 99999999));
        // mt2F.pose = new Pose2d(mt2F.pose.getTranslation(),
        // drivetrain.getState().Pose.getRotation());
        drivetrain.addVisionMeasurement(
            mt2F.pose(),
            Utils.fpgaToCurrentTime(mt2F.timestampSeconds()));
      }
      poseEstimate = drivetrain.getState().Pose;
      SmartDashboard.putData("Field", field);
      field.setRobotPose(poseEstimate);
    }
  }

  public Command updateOdomCommand(CommandSwerveDrivetrain drivetrain) {
    return run(() -> updateOdom(drivetrain));
  }

  public Pose2d getbostpose2d() {
    return poseEstimate;
  }

  @Override
  public void periodic() {
    PoseEstimate mt2F = TigerHelpers.getBotPoseEstimate("limelight-front");
    PoseEstimate mt2B = TigerHelpers.getBotPoseEstimate("limelight-back");

    tl = LimelightOneTable.getEntry("Tl").getDouble(0);
    SmartDashboard.putString("botpose F", mt2F.toString());
    SmartDashboard.putString("botpose B", mt2B.toString());

    // if (botpose.length != 0) {
    // botPoseX = botpose[0];
    // botPoseY = botpose[1];
    // botPoseZ = botpose[2];
    // roll = botpose[3];
    // pitch = botpose[4];
    // yaw = botpose[5];
    // latency = botpose[6];
    // numberOfTargetsSeen = botpose[7];
    // tagSpan = botpose[8];
    // averageDistance = botpose[9];
    // averageArea = botpose[10];
    // }
  }
}
