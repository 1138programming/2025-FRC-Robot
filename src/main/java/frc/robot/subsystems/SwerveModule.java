package frc.robot.subsystems;

import static frc.robot.Constants.SwerveDriveConstants.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private TalonFX angleMotor;
  // private CANSparkMax driveMotor;
  private TalonFX driveMotor;

  // magEncoder = absolute encoder to reset position of relative angle encoders
  private CANcoder canCoder;

  // Relative encoders are used for robot odometry and controlling speed/position
  private RelativeEncoder driveEncoder;

  private PIDController angleController;

  private double offset;

  private MotorOutputConfigs driveConfigs;

  private MotorOutputConfigs angleConfigs;

  private CurrentLimitsConfigs currentLimitsConfigs;
 
  
  
  public SwerveModule(int angleMotorID, int driveMotorID, int encoderPort, double offset, 
  InvertedValue kfrontleftdrivereversed, InvertedValue kfrontleftanglereversed) {
                        driveMotor = new TalonFX(driveMotorID);
                        angleMotor = new TalonFX(angleMotorID);
    // driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    
    angleMotor.setNeutralMode(NeutralModeValue.Brake);
    driveMotor.setNeutralMode(NeutralModeValue.Brake);

    
    driveConfigs.withInverted(kfrontleftdrivereversed);
    angleConfigs.withInverted(kfrontleftanglereversed);

    currentLimitsConfigs.withStatorCurrentLimitEnable(true).withSupplyCurrentLimit(65);

    this.driveMotor.getConfigurator().apply(driveConfigs);
    this.driveMotor.getConfigurator().apply(currentLimitsConfigs);
    this.angleMotor.getConfigurator().apply(angleConfigs);
    this.angleMotor.getConfigurator().apply(currentLimitsConfigs);

    canCoder = new CANcoder(encoderPort);

    MagnetSensorConfigs canCoderConfig = new MagnetSensorConfigs();

    double offsetToRotations = offset/360;

    canCoderConfig.AbsoluteSensorDiscontinuityPoint = 180;
    canCoderConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfig.MagnetOffset = offsetToRotations;
    canCoder.getConfigurator().apply(canCoderConfig);

    angleController = new PIDController(KAngleP, 0, KAngleD);
    angleController.enableContinuousInput(-180, 180); // Tells PIDController that 180 deg is same in both directions
  }
  
  
  public void setDesiredState(SwerveModuleState desiredState) {
    double angleMotorOutput;
    double driveMotorOutput;
    
    Rotation2d currentAngleR2D = getAngleR2D();
    desiredState = SwerveModuleState.optimize(desiredState, currentAngleR2D);
    angleMotorOutput = angleController.calculate(getAngleDeg(), desiredState.angle.getDegrees());
    
    driveMotorOutput = desiredState.speedMetersPerSecond / KPhysicalMaxDriveSpeedMPS;
    
    angleMotor.set(angleMotorOutput);
    driveMotor.set(driveMotorOutput); 
  }
  
  public void lockWheel() {
    // double angleMotorOutput;
    // if (angleMotor.getid() == KLeftFrontAngleID || angleMotor.getDeviceId() == KRightBackAngleID) {
    //   angleMotorOutput = angleController.calculate(getAngleDeg(), 45);
    // }
    // else {
    //   angleMotorOutput = angleController.calculate(getAngleDeg(), -45);
    // }
    
    // angleMotor.set(angleMotorOutput);
    // driveMotor.set(0);
  }
  
  public SwerveModulePosition getPosition() {
    SwerveModulePosition position = new SwerveModulePosition(getDriveEncoderPos(), getAngleR2D());
    return position;
  }
  
  public void stop() {
    driveMotor.set(0);
    angleMotor.set(0);
  }
  
  public void resetRelEncoders() {
    driveEncoder.setPosition(0);
  }
  
  public double getAbsoluteOffset() {
    return offset;
  }
  
  // Drive Encoder getters
  public double getDriveEncoderPos() {
    return driveMotor.getPosition().getValue().times(KDriveMotorRotToMeter).magnitude();
  }
  public double getDriveEncoderVel() {
    return driveMotor.getRotorVelocity().getValue().times(KDriveMotorRPMToMetersPerSec).magnitude();
  }
  
  // Angle Encoder getters
  public double getMagDegRaw() {
    double pos = canCoder.getAbsolutePosition().getValueAsDouble() * 360;
    return pos;
  }
  public double getAngleDeg() {
    return getMagDegRaw() % 360;
  }

  public Rotation2d getAngleR2D() {
    return Rotation2d.fromDegrees(getAngleDeg()); 
  }

  @Override
  public void periodic() {
  }
}
