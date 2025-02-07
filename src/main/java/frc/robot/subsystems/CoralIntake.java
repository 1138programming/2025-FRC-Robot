// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import static frc.robot.Constants.CoralIntakeConstants.*;

public class CoralIntake extends SubsystemBase {
  public DigitalInput CoralIntakeLimitSwitch;
  private SparkMax CoralIntakeMotor;

  public CoralIntake() {
    CoralIntakeMotor = new SparkMax(KCoralIntakeMotorId, MotorType.kBrushless);
    CoralIntakeLimitSwitch = new DigitalInput(KCoralIntakeMotorLimitSwitchPort);
  }

  public boolean getObjectPossesion(){
    return CoralIntakeLimitSwitch.get();
  }

  public void setCoralIntakeSpeed(double speed){
    if(getObjectPossesion() && speed > 0){
      CoralIntakeMotor.set(0);
    } else {
      CoralIntakeMotor.set(speed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
