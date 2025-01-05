package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Constants.LimelightConstants.*;

public class Limelight extends SubsystemBase {
  private NetworkTable aprilTagsTable;
  private NetworkTable defaultTable;
  private String aprilTagsPipeline = "AprilTags";

  private double targetFound;
  private double x;
  private double y;
  private double z;
  private double id;
  private double skew;
  private double area;
  private double pipeline;

  private double[] botPose;
  // INFO FROM botpose_wpiblue ARRAY
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
  
  public Limelight() {
    aprilTagsTable = NetworkTableInstance.getDefault().getTable("limelight");

    targetFound = 0;
    x = 0;
    y = 0;
    z = 0;
    
    id = 0;
    area = 0;
    skew = 0;
    
    botPose = new double[11];
    
    botPoseX = 1;
    botPoseY = 1;
    botPoseZ = 0;
    roll = 0;
    pitch = 0; // MAYBE SHOULD BE 15, UNSURE
    // pitch = 0;
    yaw = 0;
    latency = 0;
    numberOfTargetsSeen = 0;
    tagSpan = 0;
    averageDistance = 0;
    averageArea = 0;
  }

  @Override
  public void periodic() {
    // getting limelight networktable values

    targetFound = aprilTagsTable.getEntry("tv").getDouble(0);
    x = aprilTagsTable.getEntry("tx").getDouble(0);
    y = aprilTagsTable.getEntry("ty").getDouble(0);
    z = aprilTagsTable.getEntry("tz").getDouble(0);
    area = aprilTagsTable.getEntry("ta").getDouble(0);
    id = aprilTagsTable.getEntry("tid").getDouble(0);
    botPose = aprilTagsTable.getEntry("botpose_wpiblue").getDoubleArray(new double[11]); // Right Side of Blue Driver
                                                                                        // Station

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

  // SETTING LEDS
  public void LEDOn() {
    // Eye Protection
    getTable().getEntry("ledMode").setNumber(3); // (turns limelight on)
  }
  public void LEDOff() {
    // Eye Protection
    getTable().getEntry("ledMode").setNumber(1); // (turns limelight off)
  }
  public void LEDBlink() {
    getTable().getEntry("ledMode").setNumber(2); // (blinks limelight)
  }
  
  // botPose
  public double[] getBotPose() {
    return botPose;
  }
  public double getBotPose(int i) {
    return botPose[i];
  }  
  // botPose ARRAY VALUES
  public double getBotPoseX() {
    return botPoseX;
  }
  public double getBotPoseY() {
    return botPoseY;
  }
  public double getBotPoseZ() {
    return botPoseZ;
  }
  public double getRoll() {
    return roll;
  }
  public double getPitch() {
    return pitch;
  }
  public double getYaw() {
    return yaw;
  }
  public double getLatency() {
    return latency;
  }
  public double getNumberOfTargetsSeen() {
    return numberOfTargetsSeen;
  }
  public double getTagSpan() {
    return tagSpan;
  }
  public double getAverageDistance() {
    return averageDistance;
  }
  public double getAverageArea() {
    return averageArea;
  }

  public boolean isSpeakerAprilTagsSeen() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Blue) {
        return (getTID() == KspeakerAprilTagsBlue [0] || getTID() == KspeakerAprilTagsBlue[1]);
      } 
      else {
        return (getTID() == KspeakerAprilTagsRed[0] || getTID() == KspeakerAprilTagsRed[1]);
      }
    }
    return false;
  }
  
  
  public boolean getTargetFound() {
    return targetFound == 1;
  }
  public double getXAngle() {
    return x;
  }
  public double getYAngle() {
    return y;
  }
  public double getZAngle() {
    return z;
  }
  public double getArea() {
    return area;
  }
  public NetworkTable getTable() {
    return defaultTable;
  }
  public String getTableString() {
    if (pipeline == 0) {
      return "AprilTags";
    }
    return "Tape";
  }
  public double getPipeline() {
    return pipeline;
  }
  public void setPipeline(int pipeline) {
    this.pipeline = pipeline;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline); // (turns
                                                                                                      // limelight on)
  }
  /**
   * Get ID of nearest AprilTag
   */
  public double getTID() {
    return id;
  }
  public double getSkew() {
    return skew;
  }
}