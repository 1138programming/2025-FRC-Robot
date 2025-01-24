package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ArmConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Arm extends SubsystemBase {

    private TalonFX tiltMotor;
    private DutyCycleEncoder tiltThroughBoreEncoder;

    private PIDController tiltController;
    private PIDController absoluteTiltController; 


    public Arm() {

        tiltMotor = new TalonFX(ArmConstants.KTiltArmId);

        //Set up ThroughBore encoder
        tiltThroughBoreEncoder = new DutyCycleEncoder(ArmConstants.KTiltThroughEncoderId, 4.0, 2.0);
        tiltThroughBoreEncoder.reset(); //The method reset() is undefined for the type DutyCycleEncoder

            




        tiltController = new PIDController(0,0,0);
        absoluteTiltController = new PIDController(0, 0, 0); //test and see what values work
    }

    @Override
    public void periodic() {

    }

    public double getTiltEncoder () {
        return tiltThroughBoreEncoder.get() % 360;
    }

    public void tiltArm() {
    }
}
