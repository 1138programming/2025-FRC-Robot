package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;

//import com.ctre.motorcontrol.can.FalconSRX;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

//TalonFX myMotor = new TalonFX(10); - motor

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
    private DigitalInput toplimitSwitch = new DigitalInput(0);
    private DigitalInput bottomlimitSwitch = new DigitalInput(1);
    private double startingAngle;
    private PositionVoltage m_request;
    private MagnetSensorConfigs canCoderConfig = new MagnetSensorConfigs();
    // in init function, set slot 0 gains

    public Lift() {
        // CANCoder
        spinLiftCANCoder = new CANcoder(KLiftCANCoderID);

        double offsetToRotations = 0;


        // canCoderConfig.AbsoluteSensorDiscontinuityPoint = AbsoluteSensorDiscontinuityPoint;
        // canCoderConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        // canCoderConfig.MagnetOffset = offsetToRotations;
        spinLiftCANCoder.getConfigurator().apply(canCoderConfig);
        var slot0Configs = new Slot0Configs();
        liftMotor.getConfigurator().apply(liftMotorConfig);
        liftMotor = new TalonFX(KSpinMotorID);
        slot0Configs.kP = 0; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output

        liftMotor.getConfigurator().apply(slot0Configs);
        m_request = new PositionVoltage(0).withSlot(0);

    }

    // Limit Switches
    @Override
    public void periodic() {

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
    public void SetLiftToPoint(double setpoint) {
        // create a position closed-loop request, voltage output, slot 0 configs

        // set position to 10 rotations (degrees/360!)
        liftMotor.setControl(m_request.withPosition(setpoint));
    }

    public double getLiftCANCoder() {
        double angle = spinLiftCANCoder.getPosition().getValueAsDouble() * 360 + startingAngle;
        // startingAngle = 0 - spinLiftCANCoder.getPosition().getValueAsDouble()*360;
        return angle;

    }
}
