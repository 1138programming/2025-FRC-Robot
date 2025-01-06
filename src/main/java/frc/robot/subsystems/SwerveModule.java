package frc.robot.subsystems;

import static frc.robot.Constants.SwerveDriveConstants.*;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private CANSparkMax angleMotor;
  // private CANSparkMax driveMotor;
  private CANSparkFlex driveMotor;

  // magEncoder = absolute encoder to reset position of relative angle encoders
  private CANcoder canCoder;

  // Relative encoders are used for robot odometry and controlling speed/position
  private RelativeEncoder driveEncoder;

  private PIDController angleController;

  private double offset;
  
  public SwerveModule(int angleMotorID, int driveMotorID, int encoderPort, double offset, 
                      boolean driveMotorReversed, boolean angleMotorReversed) {
    angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
    driveMotor = new CANSparkFlex(driveMotorID, MotorType.kBrushless);
    // driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

    angleMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setIdleMode(IdleMode.kBrake);

    this.angleMotor.setInverted(angleMotorReversed);
    this.driveMotor.setInverted(driveMotorReversed);
    
    this.driveMotor.setSmartCurrentLimit(KDriveMotorCurrentLimit);
    this.angleMotor.setSmartCurrentLimit(KAngleMotorCurrentLimit);

    canCoder = new CANcoder(encoderPort);

    MagnetSensorConfigs canCoderConfig = new MagnetSensorConfigs();

    double offsetToRotations = offset/360;

    canCoderConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    canCoderConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfig.MagnetOffset = offsetToRotations;
    canCoder.getConfigurator().apply(canCoderConfig);

    // driveEncoder = driveMotor.getExternalEncoder(Type.kQuadrature, 1);
    driveEncoder = driveMotor.getEncoder();
    
    driveEncoder.setPositionConversionFactor(KDriveMotorRotToMeter);
    driveEncoder.setVelocityConversionFactor(KDriveMotorRPMToMetersPerSec);

    angleController = new PIDController(KAngleP, KAngleI, KAngleD);
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
    double angleMotorOutput;
    if (angleMotor.getDeviceId() == KLeftFrontAngleID || angleMotor.getDeviceId() == KRightBackAngleID) {
      angleMotorOutput = angleController.calculate(getAngleDeg(), 45);
    }
    else {
      angleMotorOutput = angleController.calculate(getAngleDeg(), -45);
    }
    
    angleMotor.set(angleMotorOutput);
    driveMotor.set(0);
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
    return driveEncoder.getPosition();
  }
  public double getDriveEncoderVel() {
    return driveEncoder.getVelocity();
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
