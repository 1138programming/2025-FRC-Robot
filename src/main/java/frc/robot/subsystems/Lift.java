package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.LiftConstants.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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

    private CANcoder spinLiftCANCoder;
    private CANcoderConfiguration canCoderConfig;

    private DigitalInput toplimitSwitch = new DigitalInput(0);
    private DigitalInput bottomlimitSwitch = new DigitalInput(1);

    private double startingAngle;

    private VoltageOut voltageController;

    private PositionVoltage m_request;
    // in init function, set slot 0 gains

    private PositionVoltage m_PositionVoltage;
    
    private TrapezoidProfile m_TrapezoidProfile;

    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;



    private final SysIdRoutine m_sysIdRoutineLift = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdLift_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setMotorVoltage(volts),
                    null,
                    this));

    public Lift() {
        // CANCoder
        spinLiftCANCoder = new CANcoder(KLiftCANCoderID);
        canCoderConfig = new CANcoderConfiguration();
        // canCoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        spinLiftCANCoder.getConfigurator().apply(canCoderConfig);

        voltageController = new VoltageOut(0);
        // Motor
        liftMotor = new TalonFX(KSpinMotorID);

        liftMotorConfig = new TalonFXConfiguration();
        liftMotorConfig.Feedback.FeedbackRemoteSensorID = spinLiftCANCoder.getDeviceID();
        liftMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        liftMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
        liftMotorConfig.Feedback.RotorToSensorRatio = 46.67;

        liftMotor.getConfigurator().apply(liftMotorConfig);

        m_PositionVoltage = new PositionVoltage(0).withEnableFOC(true);
        m_TrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(KMaxVoltage, KMaxAcceleration));

        // pid config
        var slot0Configs = new Slot0Configs()
                .withKP(0).withKI(0).withKD(0)
                .withKS(0).withKV(2).withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

        liftMotor.getConfigurator().apply(slot0Configs);

        m_request = new PositionVoltage(0).withSlot(0);

        startingAngle = 0;

        m_goal = new TrapezoidProfile.State(0, 0); 

        m_setpoint = new TrapezoidProfile.State();
    }

    public void setMotorVoltage(Voltage volts) {
        if (volts.magnitude() > 0) {
            if (toplimitSwitch.get()) {
                liftMotor.setControl(voltageController.withOutput(volts));
            } else {
                liftMotor.setControl(voltageController.withOutput(0));
            }
        } else {
            if (bottomlimitSwitch.get()) {
                liftMotor.setControl(voltageController.withOutput(0));
            } else {
                liftMotor.setControl(voltageController.withOutput(volts));
            }
        }
    }

    public void setMotorSpeed(double speed) {
        if (speed > 0) {
            if (toplimitSwitch.get()) {
                liftMotor.set(speed);
            } else {
                liftMotor.set(speed);
            }
        } else {
            if (bottomlimitSwitch.get()) {
                liftMotor.set(0);
            } else {
                liftMotor.set(speed);
            }
        }
    }

    public void MoveLiftToSetPositionCTRE(double position) {
        //Should call setArmPosition encorperating steamdeck button values to correspond to set points
        m_goal = new TrapezoidProfile.State(position, 0); //new goal position
        m_setpoint = m_TrapezoidProfile.calculate(0.020, m_setpoint, m_goal); //calculates the new setpoint based on the new goal

        m_PositionVoltage.Position = m_setpoint.position;
        m_PositionVoltage.Velocity = m_setpoint.velocity;
        liftMotor.setControl(m_PositionVoltage);
    }

    // PID
    public void MoveLiftToSetPositionWPI(double setpoint) {
        // In rotations (degrees/360)
        liftMotor.setControl(m_request.withPosition(setpoint));
    }

    public double getLiftCANCoder() {
        double angle = spinLiftCANCoder.getPosition().getValueAsDouble() * 360 + startingAngle;
        return angle;

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

    }
}
