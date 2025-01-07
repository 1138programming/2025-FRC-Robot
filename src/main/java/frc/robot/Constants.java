// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix6.signals.InvertedValue;

// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statical0ly import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    // Controller Ports (check in Driver Station, IDs may be different for each
    // compStreamDeckuter)
    public static final int KLogitechPort = 0;
    public static final int KXboxPort = 1;
    public static final int KCompStreamDeckPort = 2;
    public static final int KTestingStreamDeckPort = 3;
    public static final int KAutonTestingStreamDeckPort = 4;

    // Deadzone
    public static final double KDeadZone = 0.05;

    // Joystick Axis IDs
    public static final int KLeftXAxis = 0;
    public static final int KLeftYAxis = 1;
    public static final int KRightXAxis = 2;
    public static final int KRightYAxis = 3;

    // Joystick Axis IDs
    public static final int KXboxLeftYAxis = 1;
    public static final int KXboxRightYAxis = 5;
    public static final int KXboxLeftXAxis = 0;
    public static final int KXboxRightXAxis = 4;

    // Logitech Button Constants
    public static final int KLogitechButtonX = 1;
    public static final int KLogitechButtonA = 2;
    public static final int KLogitechButtonB = 3;
    public static final int KLogitechButtonY = 4;
    public static final int KLogitechLeftBumper = 5;
    public static final int KLogitechRightBumper = 6;
    public static final int KLogitechLeftTrigger = 7;
    public static final int KLogitechRightTrigger = 8;

    // Xbox Button Constants
    public static final int KXboxButtonA = 1;
    public static final int KXboxButtonB = 2;
    public static final int KXboxButtonX = 3;
    public static final int KXboxButtonY = 4;
    public static final int KXboxLeftBumper = 5;
    public static final int KXboxRightBumper = 6;
    public static final int KXboxSelectButton = 7;
    public static final int KXboxStartButton = 8;
    public static final int KXboxLeftTrigger = 2;
    public static final int KXboxRightTrigger = 3;
  }

  public static class LimelightConstants {
    public static final double KlimelightMountAngleDegrees = 25.0; // Neeeds to be changed
    public static final double KlimelightMountHeight = 0.508; 
    public static final double KShooterTiltMountHeight = 0.2286; 
    public static final double KspeakerHeight = 2.159;  
    public static final double[] KSpeakerCoordinatesBlue = new double[] { 0, 5.547868 }; // (X,Y) of the center
    // public static final double[] KSpeakerCoordinatesBlue = new double[] { 0, 5.5474108 }; // (X,Y) of the center
                                                                                          // aprilTag
    public static final double[] KspeakerAprilTagsBlue = new double[] { 7, 8 }; // Right To Left
    public static final double[] KSpeakerCoordinatesRed = new double[] { 16.58, 5.547868 }; // (X,Y) of the center
    // public static final double[] KSpeakerCoordinatesRed = new double[] { 16.52, 5.5474108 }; // (X,Y) of the center
                                                                                              // aprilTag
    public static final double[] KspeakerAprilTagsRed = new double[] { 3, 4 }; // Right To Left
    public static final double KlimeLightRotP = 0.0167;
    public static final double KlimeLightRotI = 0;
    public static final double KlimeLightRotD = 0;
    public static final double KlimeLightDriveP = 0;
    public static final double KlimeLightDriveI = 0;
    public static final double KlimeLightDriveD = 0;
    public static final double KLimelightAngleDeadzone = 1;
    public static final double KaprilTagOffset = 20;
    public static final PIDController KlimelightrotControl = new PIDController(KlimeLightRotP, KlimeLightRotI,
    KlimeLightRotD);
    public static final PIDController KBaseController = new PIDController(KlimeLightDriveP, KlimeLightDriveI,
    KlimeLightDriveD);
  }

  public static class LEDConstants
  {
    public static final int KLEDPort = 9; //placeholder
    public static final int KLEDBuffer = 60;
  }

  public static class SwerveDriveConstants {
    // Drive motors
    public static final int KLeftFrontDriveID = 2; // SparkFlex + Vortex
    public static final int KRightFrontDriveID = 4; // SparkFlex + Vortex
    public static final int KLeftBackDriveID = 6; // SparkFlex + Vortex
    public static final int KRightBackDriveID = 8; // SparkFlex + Vortex

    // Angle motors
    public static final int KLeftFrontAngleID = 1; // SparkMax + NEO
    public static final int KRightFrontAngleID = 3; // SparkMax + NEO
    public static final int KLeftBackAngleID = 5; // SparkMax + NEO
    public static final int KRightBackAngleID = 7; // SparkMax + NEO

    // CanCoders
    public static final int KLeftFrontEncoderID = 1;
    public static final int KRightFrontEncoderID = 2;
    public static final int KLeftBackEncoderID = 3;
    public static final int KRightBackEncoderID = 4;

    // Swerve Angle PID
    public static final double KAngleP = 0.006;
    public static final double KAngleD = 0;

    // Drive Angle PID
    public static final double KDriveP = 0;
    public static final double KDriveD = 0;

    // Swerve Current Limits
    public static final int KDriveMotorCurrentLimit = 70;
    public static final int KAngleMotorCurrentLimit = 40;

    // Motor Info
    public static final double KNeoMaxRPM = 5676;
    public static final double KNeoVortexMaxRPM = 6784;
    public static final int KVortexEncoderTicksPerRevolution = 7168;
    public static final double KKrakenX60MaxRPM = 6000;

    // Robot Specs
    public static final double KDriveMotorToWheelGearRatio = 1 / 5.36; 
    public static final double KWheelDiameterMeters = 0.1016;
    public static final double KDriveMotorRotToMeter = KDriveMotorToWheelGearRatio * KWheelDiameterMeters * Math.PI;
    public static final double KDriveMotorRPMToMetersPerSec = KDriveMotorRotToMeter / 60;
    public static final double KPhysicalMaxDriveSpeedMPS = KNeoVortexMaxRPM * KDriveMotorRPMToMetersPerSec;
    public static final double KWheelRadialDistanceFromCenter = 0.4953;
    public static final double KWheelDistanceFromCenter = 0.34925;

    // Swerve Wheel X and Y Coordinates for Driving
    public static final Translation2d KFrontLeftLocation = new Translation2d(
        KWheelDistanceFromCenter, KWheelDistanceFromCenter);
    public static final Translation2d KFrontRightLocation = new Translation2d(
        KWheelDistanceFromCenter, -KWheelDistanceFromCenter);
    public static final Translation2d KBackLeftLocation = new Translation2d(
        -KWheelDistanceFromCenter, KWheelDistanceFromCenter);
    public static final Translation2d KBackRightLocation = new Translation2d(
        -KWheelDistanceFromCenter, -KWheelDistanceFromCenter);

    // Max Speeds
    public static final double KMaxAcceleration = 8;
    public static final double KMaxAngularSpeed =   10;
    // public static final double KMaxAngularSpeed = 3.5;

    // Offsets
    //  Meow (Gray Bot)
    public static final double KFrontLeftOffset = 0;
    public static final double KFrontRightOffset = 0;
    public static final double KBackLeftOffset = 0;
    public static final double KBackRightOffset = 0;
    // public static final double KFrontLeftOffset = -104.326171875;
    // public static final double KFrontRightOffset = -101.69;
    // public static final double KBackLeftOffset = 101.34;
    // public static final double KBackRightOffset = -34.45;

    // Drive Motor Reversals
    public static final InvertedValue KFrontLeftDriveReversed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue KFrontRightDriveReversed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue KBackLeftDriveReversed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue KBackRightDriveReversed = InvertedValue.Clockwise_Positive;

    // Angle Motor Reversals
    public static final InvertedValue KFrontLeftAngleReversed = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue KFrontRightAngleReversed = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue KBackLeftAngleReversed = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue KBackRightAngleReversed = InvertedValue.CounterClockwise_Positive;

    // Swerve CanCoder Reversals
    public static final boolean KFrontLeftDriveEncoderReversed = false;
    public static final boolean KFrontRightDriveEncoderReversed = false;
    public static final boolean KBackLeftDriveEncoderReversed = false;
    public static final boolean KBackRightDriveEncoderReversed = false;

    // Low and high percent: sets max speed of drivetrain for driver
    public static final double KBaseDriveLowPercent = 0.25;
    public static final double KBaseDriveMidPercent = 0.5;
    public static final double KBaseDriveMaxPercent = 1;
    
    public static final double KBaseRotLowPercent = 0.75;
    public static final double KBaseRotMidPercent = 1;
    public static final double KBaseRotMaxPercent = 1.5;

    public static final double KRotationP = 0.02;
    public static final double KRotationI = 0;
    public static final double KRotationD = 0;
    
    public static final double KTranslationP = 20;
    public static final double KTranslationI = 1.5;
    public static final double KTranslationD = 0;

    public static final double KDrivingPidP = 0.008;
    public static final double KDrivingPidI = 0;
    public static final double KDrivingPidD = 0;
    // Auton Config
    // public static final HolonomicPathFollowerConfig KPathFollowerConfig = new HolonomicPathFollowerConfig(
    //   new PIDConstants(KTranslationP, KTranslationI, KTranslationD), // Translation constants
    //   new PIDConstants(3, 0, 0), // Rotation constants
    //   KPhysicalMaxDriveSpeedMPS,
    //   KWheelRadialDistanceFromCenter, // Drive base radius (distance from center to furthest module)
    //   new ReplanningConfig()
    // );
  }

  public static class IntakeConstants {
    // Motor ID
    public static final int KIntakeMotorID = 12;

    // Motor setup
    public static final boolean KIntakeMotorIsInverted = false;
    public static final int KIntakeMotorCurrentLimit = 40;

    // Motor speed
    public static final double KIntakeMotorSpeed = 1;
  }

  public static class ShooterTiltConstants {
    // Motor ID
    public static final int KShooterTiltMotorID = 16;
    // Encoder IDs
    public static final int KShooterTiltEncoderID = 5; // CANCoder
    public static final int KShooterTiltAbsoluteEncoderID = 2; // Throughbore
    
    // CANCoder offset
    public static final double KShooterTiltEncoderOffset = 0; // CANCoder
    public static final double KShooterTiltAbsoluteOffset = 0.0435; // Throughbore
    // public static final double KShooterTiltAbsoluteOffset = 50.56/360; // Throughbore

    // Gear Ratio
    public static final double KTiltMotorToSwivelGearRatio = 0.85714286; // 36 to 42
    public static final double KTiltMotorToSwivelGearRatioBackwards = 1.1667; // 42 to 36

    // Motor Speed
    public static final double KShooterTiltMotorSpeed = 0.25;

    public static final double KShooterTiltDistanceOffGround = 0.0508;

    // Untuned - PID Constants
    public static final double KShooterTiltControllerP = 0.010;
    // public static final double KShooterTiltControllerP = 0.012;
    public static final double KShooterTiltControllerI = 0;
    public static final double KShooterTiltControllerD = 0.0001;

    public static final double KShooterTiltAbsoluteControllerP = 0.01;
    public static final double KShooterTiltAbsoluteControllerI = 0;
    public static final double KShooterTiltAbsoluteControllerD = 0;
    // public static final double KShooterTiltAbsoluteControllerP = 0.015;
    // public static final double KShooterTiltAbsoluteControllerI = 0;
    // public static final double KShooterTiltAbsoluteControllerD = 0.0001;
    
    // public static final double KShooterTiltControllerPUp = 0.01; 
    public static final double KShooterTiltControllerShootP = 0.018; // holds tilt in place while shooting
    // Testing
    public static final double kShooterTiltDeadZone = 1;
    // public static 
    public static final double KShooterTiltAngleOffset = -7;

    public static final double kShooterTiltUpPos = 76.7;
    public static final double KShooterTiltAmpAngle = 55;
    public static final double KShooterTiltSubAngle = 63;
    public static final double KShooterTiltSource = 57.9;
    public static final double KShooterTiltPodiumAngle = 34.5;
    public static final double KShooterTiltAuton1Angle = 50.6;
    public static final double KShooterTiltAuton2Angle = 47.8;
    public static final double KShooterTiltBottomAngle = 30;

    public static final double KShooterTiltCloseAimOffset = 0.8;
    public static final double KShooterTiltMediumAimOffset = -1.15;
    public static final double KShooterTiltFarAimOffset = -6;
    // public static final double KShooterTiltWingAimOffset = -2;

    // This array must be sorted for the shooter tilt functionality to work!!!!
    public static final double[][] KShooterTiltAngles =
    {
      {17.71, 18,   20,   25,    30,    35,    40,   45,     50,    55,    60,     65,  70,     75,     80}, 
      {0,     12.6, 33.8, 56.78, 71.37, 83.12, 93.5, 103.14, 112.4, 121.5, 130.64, 140, 149.86, 160.72, 173.93}
    };
    // public static final double[][] KShooterTiltAngles =
    // {
    //   {17.71, 18,   20,   25,    30,    35,    40,   45,     50,    55,    60,     65,  70,     75,     80,     83.92}, 
    //   {0,     12.6, 33.8, 56.78, 71.37, 83.12, 93.5, 103.14, 112.4, 121.5, 130.64, 140, 149.86, 160.72, 173.93, 196.07}
    // };
    public static final int KShooterTiltAnglesMaxIndex = KShooterTiltAngles[0].length - 1;
    
  }
  
  public static class FlywheelConstants{
    // Motor IDs
    public static final int KShooterUpperMotor = 14;
    public static final int KShooterLowerMotor = 15;

    // reversed motor
    public static final boolean KFlywhelUpperMotorReversed = true;

    //Motor speeds
    public static final double KFlywheelFullSpeed = 1; 
    public static final double KFlywheelFarSpeed = 0.8; 
    public static final double KFlywheelCloseSpeed = 0.7; 
    public static final double KFlywheelSlowSpeed = 0.3; 
    // public static final double KFlywheelLowSpeed = 0.185;
    public static final double KFLywheelAmpSpeed = 0.35;
    // public static final double KFLywheelAmpSpeed = 0.40;

    public static final double KFlywheelCloseSpeedMaxDistance = 3.75;
    public static final double KFlywheelTiltUpDistance = 3;
    
    public static final double KFlywheelVelocity = 5000;
    
    // PID Controller
    public static final double KFlywheelP = 0; 
    public static final double KFlywheelI = 0; 
    public static final double KFlywheelD = 0; 
  }

  public static class IndexerConstants{
    public static final double KIndexerMotorSpeed = 0.2;
    public static final double KIndexerSlowSpeed = 0.2;
    public static final double KIndexerFastSpeed = 0.5;

    public static final int KIndexerBBreakerTopID = 7;
    public static final int KIndexerBBreakerBottomID = 9;

    public static final int KIndexerMotorID = 13;
  }

  public static class HangConstants {
    public static final int KHangMotorID = 17;

    public static final int KLaserCanID = 5;

    public static final double KHangMotorSpeedUp = 0.5;
    public static final double KHangMotorSpeedDown = -0.5;

    public static final int KHangLimitSwitchDown = 2;
    public static final int KHangLimitSwitchUp = 3;

    public static final int KHangSetPositionUp = 10;
    public static final int KHangSetPositionDown = -10;

    // Piston Pneumatics double Constants
    public static final int KHangPistonLeftForwardID = 2;
    public static final int KHangPistonLeftBackwardID = 0;
    public static final int KHangPistonRightForwardID = 3;
    public static final int KHangPistonRightBackwardID = 1;
  }

  public static class TrapConstants{
    public static final int KTrapRollerMotorID = 18;
    public static final int KTrapWristMotorID = 19;
    public static final int KTrapIRID = 2;
    public static final int KPotentiometerID = 1;

    public static final double KTrapRollersForwardSpeed = 0.25;
    public static final double KTrapWristUpSpeed = 0.5;
    public static final double KTrapWristDownSpeed = -0.5;
    
    public static final double KAnalogPotentiometerSensorRange  = 270;
    public static final double KAnalogPotentiometerSensorOffset  = 0;
    public static final double KTrapWristAmp = 198;
    public static final double KTrapWristSource = 213;
    public static final double KTrapWristStow = 30;
    // public static final double KTrapPotentiometerSetpoint = 170;

    public static final double KTrapControllerP = 0.006;
    public static final double KTrapControllerI = 0;
    public static final double KTrapControllerD = 0;
  }
  
  public static class CameraConstants {
    public static final int KCameraPivotServoID = 1;
  }
}
