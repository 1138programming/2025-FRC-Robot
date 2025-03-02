// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import static frc.robot.Constants.DeviceConstants.*;
import static frc.robot.Constants.HangConstants.*;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/*In entire lift:
2 limit switches, 
- bottom one autonomus direct
- top one, also there
1 cancoder at bottom
1 Talon to rotate string
*/

public class Hang extends SubsystemBase {
  /** Creates a new Hang. */
  private TalonFX tiltMotor;
  private VoltageOut voltageController;

  private Servo lock;

 DutyCycleEncoder HangEncoder;
  private final SysIdRoutine m_sysIdRoutineHang = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(5), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdLift_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setHangVoltage(volts),
                    null,
                    this));


  public Hang() {
    tiltMotor = new TalonFX(KHangMotorId);
    HangEncoder = new DutyCycleEncoder(KHangThroughEncoderId, KHangThroughEncoderFullRotationValue, KHangThroughEncoderZeroPosition);
    voltageController = new VoltageOut(0).withEnableFOC(false);

    lock = new Servo(KHangLock);


  }
  public void tiltHang() {

  }
  public void tiltHangToPos() {
    
  }

  public void setlock(double pos) {
    lock.set(pos);
  }
  public void setHangVoltage(Voltage Volt) {
    tiltMotor.setControl(voltageController.withOutput(Volt));
  }
   public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineHang.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineHang.dynamic(direction);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
