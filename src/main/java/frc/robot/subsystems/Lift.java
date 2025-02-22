package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.LiftConstants.*;
import static frc.robot.Constants.LiftConstants.LiftPositionConstants.KLiftControlP;
import static frc.robot.Constants.DeviceConstants.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

/*In entire lift:
2 limit switches, 
- bottom one autonomus direct
- top one, also there
1 cancoder at bottom
1 Talon to rotate string
*/

public class Lift extends SubsystemBase {
    private TalonFX liftMotor;
    private TalonFXConfiguration liftMotorConfig;
    private MotorOutputConfigs motorOutputConfigs;

    private CANcoder liftCANCoder;
    private CANcoderConfiguration canCoderConfig;

    private DigitalInput toplimitSwitch;
    private DigitalInput bottomlimitSwitch;

    private double startingAngle;

    private VoltageOut voltageController;

    private PositionVoltage m_request;
    // in init function, set slot 0 gains

    private PositionVoltage m_PositionVoltage;

    private TrapezoidProfile m_TrapezoidProfile;

    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;

    private boolean manualControl = false;

    private final SysIdRoutine m_sysIdRoutineLift = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(5), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdLift_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setMotorVoltage(volts),
                    null,
                    this));

    public Lift() {
        // CANCoder
        liftCANCoder = new CANcoder(KLiftCANCoderID);
        canCoderConfig = new CANcoderConfiguration();

        toplimitSwitch = new DigitalInput(KLiftTopLimitSwitch);
        bottomlimitSwitch = new DigitalInput(KLiftBottomLImitSwitch);

        // canCoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoderConfig.MagnetSensor.MagnetOffset = -1.87;
        liftCANCoder.getConfigurator().apply(canCoderConfig);

        // Motor
        liftMotor = new TalonFX(KLiftMotorID);

        liftMotorConfig = new TalonFXConfiguration();
        liftMotorConfig.Feedback.FeedbackRemoteSensorID = liftCANCoder.getDeviceID();
        liftMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        liftMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
        liftMotorConfig.Feedback.RotorToSensorRatio = 46.67;
        liftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        liftMotor.getConfigurator().apply(liftMotorConfig);

        m_PositionVoltage = new PositionVoltage(0).withEnableFOC(true);
        m_TrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(KMaxVoltage, KMaxAcceleration));

        voltageController = new VoltageOut(0).withEnableFOC(false);

        // pid config
        var slot0Configs = new Slot0Configs()
                .withKP(8).withKI(0).withKD(0)
                .withKS(3.2186).withKV(1.6689).withKA(0).withKG(0.34152)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

        liftMotor.getConfigurator().apply(slot0Configs);

        m_request = new PositionVoltage(0).withSlot(0);

        startingAngle = 0;

        m_goal = new TrapezoidProfile.State(0, 0);

        m_setpoint = new TrapezoidProfile.State();
    }

    public void setMotorVoltage(Voltage volts) {
        liftMotor.setControl(voltageController.withOutput(volts));
    }

    public void setMotorSpeed(double speed) {
        if (speed > 0) {
            if (!toplimitSwitch.get()) {
                liftMotor.set(0);
            } else {
                liftMotor.set(speed);
            }
        } else {
            if (!bottomlimitSwitch.get()) {
                liftMotor.set(0);
            } else {
                liftMotor.set(speed);
            }
        }
    }

    public void stopLift() {
        liftMotor.set(0);
    }

    public void MoveLiftToSetPositionCTRE(double position) {
        // Should call setArmPosition encorperating steamdeck button values to
        // correspond to set point
        // Should call setArmPosition encorperating steamdeck button values to
        // correspond to set points
        m_goal = new TrapezoidProfile.State(position, 0); // new goal position
        m_setpoint = m_TrapezoidProfile.calculate(0.020, m_setpoint, m_goal); // calculates the new setpoint based on
                                                                              // the new goal

        m_PositionVoltage.Position = m_setpoint.position;
        m_PositionVoltage.Velocity = m_setpoint.velocity;
        if (!manualControl) {
            liftMotor.setControl(m_PositionVoltage);
        }
    }

    // PID
    public void MoveLiftToSetPositionWPI(double setpoint) {
        if (!manualControl) {
            liftMotor.setControl(m_request.withPosition(setpoint));
        }
    }

    public double getLiftCANCoder() {
        double angle = liftCANCoder.getPosition().getValueAsDouble() * 360;
        return angle;
    }

    public void setManualControl() {
        manualControl = !manualControl;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineLift.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineLift.dynamic(direction);
    }

    // Limit Switches
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Lift Top", toplimitSwitch.get());
        SmartDashboard.putBoolean("Lift Bottom", bottomlimitSwitch.get());
        SmartDashboard.putNumber("Lift encoder", liftCANCoder.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Lift Manual Control", manualControl);

    }
}
