package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DigitalInput; //limit switch


/*
 * Designer intends for arm to be prefrebaly human controlled and to have specific set points:
 * 
 * -High
 * -Mid
 * -Low
 * -Bottom
 * -Store
 * -Intake off ground
 * -Intake from player
 */
public class Arm extends SubsystemBase {

    private TalonFX tiltMotor;

    private Slot0Configs Slot0Configs;

    private PIDController armPid;



    final private PositionVoltage m_PositionVoltage;
    final private DutyCycleOut m_PositionDutyCycle;
    final private TrapezoidProfile m_TrapezoidProfile; //used for motion profiling
    private TrapezoidProfile.State m_goal; //used for motion profiling -> the current target position
    private TrapezoidProfile.State m_setpoint; //used for motion profiling -> the current position to use for PID control

    private DutyCycleEncoder tiltThroughBoreEncoder;

    private DigitalInput hallSensorTop;
    private DigitalInput hallSensorBottom;
    
    public Arm() {

        tiltMotor = new TalonFX(KTiltArmId);
        tiltThroughBoreEncoder = new DutyCycleEncoder(KTiltThroughEncoderId, KTiltThroughEncoderFullRotationValue, KTiltThroughEncoderZeroPosition);

        //WPI Pid
            armPid = new PIDController(KArmControlP, KArmControlI, KArmControlD);
            m_PositionDutyCycle = new DutyCycleOut(0).withEnableFOC(true);

        //CTRE pid
            Slot0Configs = new Slot0Configs();
            //config.Slot0.kS = ArmConstants.KArmControlS; } for later use if wish to add values to overcome static friction
            //config.Slot0.kV = ArmConstants.KArmControlV; } 
            Slot0Configs.kP = KArmControlP;
            Slot0Configs.kI = KArmControlI;   
            Slot0Configs.kD = KArmControlD;   

            m_PositionVoltage = new PositionVoltage(0).withEnableFOC(true);
            m_TrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(KMaxVoltage, KMaxAcceleration));
            //set first goal to stored position of arm, will change later upon use input

            //TODO: later on, might set potition to throughborenecoder and reset the position for 
        
            //accurate position tracking throughout tournaments
            m_goal = new TrapezoidProfile.State(0, 0); 
            m_setpoint = new TrapezoidProfile.State();


            tiltMotor.getConfigurator().apply(Slot0Configs);  //apply the configuration to the motor, add PID values

        tiltMotor.setNeutralMode(NeutralModeValue.Brake); //set the motor to brake mode so arm is precise


        hallSensorTop = new DigitalInput(KHallSensorTopId);
        hallSensorBottom = new DigitalInput(KHallSensorBottomId);  
    }

    @Override
    public void periodic() {}

    public void tiltArmManually(DoubleSupplier speed) {
        
        //check if for top sensor
        if (hallSensorTop.get() && speed.getAsDouble() > 0) {
            speed = () -> 0;
        }

        //check if for bottom sensor
        if (hallSensorBottom.get() && speed.getAsDouble() < 0) {
            speed = () -> 0;
            
        }

        tiltMotor.set(speed.getAsDouble());

        // double currentPosition = getTiltEncoder();
        // m_setpoint = new TrapezoidProfile.State(currentPosition, 0);
    }

    //TODO: make sure arm position does not exceed physical limits of arm / limit switch
    //position should be in rotations
    //Set up steamdeck buttons to correspond to values
    public void tiltArmToSetPositionCTRE(double position) {
        //Should call setArmPosition encorperating steamdeck button values to correspond to set points
        m_goal = new TrapezoidProfile.State(position, 0); //new goal position
        m_setpoint = m_TrapezoidProfile.calculate(0.020, m_setpoint, m_goal); //calculates the new setpoint based on the new goal

        m_PositionVoltage.Position = m_setpoint.position;
        m_PositionVoltage.Velocity = m_setpoint.velocity;
        tiltMotor.setControl(m_PositionVoltage);
    }

    public void tiltArmToSetPositionWPI(double position) {
        tiltMotor.setControl(m_PositionDutyCycle.withOutput(armPid.calculate(tiltThroughBoreEncoder.get(), position)));
    }

    //TODO: check motor direction and adjust condition accordingly
    //Right now motor spins positively in the upward direction
    //TODO: prevents arm from moving past hall sensors


    public boolean isAtSetPosition() {
        return Math.abs(m_setpoint.position - m_goal.position) <= KArmDeadZone; //need to tune
    }

    //will remind m_setpoint of current position using encoder
    //In case manual confuses the PID
    public void setPositionFromEncoder() {
        m_setpoint = new TrapezoidProfile.State(getTiltEncoder(), 0);
    }

    public double getTiltEncoder() {
        return tiltThroughBoreEncoder.get();
    }

    public void stop() {
        tiltMotor.set(0);
    }
}
