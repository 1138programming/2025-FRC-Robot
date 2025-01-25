package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

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

    private PositionVoltage m_request;
    // in init function, set slot 0 gains

    public Lift() {
        // CANCoder
        spinLiftCANCoder = new CANcoder(KLiftCANCoderID);
        canCoderConfig = new CANcoderConfiguration();
        // canCoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        spinLiftCANCoder.getConfigurator().apply(canCoderConfig);
        //Weird 90, 180, -90, 0 rotation to help motor get to set point quicker!


        // Motor
        liftMotor = new TalonFX(KSpinMotorID);

        liftMotorConfig = new TalonFXConfiguration();
        liftMotorConfig.Feedback.FeedbackRemoteSensorID = spinLiftCANCoder.getDeviceID();
        liftMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        liftMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
        //Sensor to mech ratio, sensor on mech so just 1:1!
        liftMotorConfig.Feedback.RotorToSensorRatio = 46.67;
        //Motor to sensor ratio!
        
        liftMotor.getConfigurator().apply(liftMotorConfig);

        //pid config
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output

        liftMotor.getConfigurator().apply(slot0Configs);

        m_request = new PositionVoltage(0).withSlot(0);

        startingAngle = 0;
    }

    
    public void setMotorSpeed(double speed) {
        if (speed > 0) {
            if (toplimitSwitch.get()) {
                liftMotor.set(0);
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
    
    // PID
    public void SetLiftToPos(double setpoint) {
        //In rotations (degrees/360)
        liftMotor.setControl(m_request.withPosition(setpoint));
    }
    
    public double getLiftCANCoder() {
        double angle = spinLiftCANCoder.getPosition().getValueAsDouble() * 360 + startingAngle;
        return angle;
        
    }

    // Limit Switches
    @Override
    public void periodic() {

    }
}
