package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.motorcontrol.can.TalonFX;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


//TalonFX myMotor = new TalonFX(10); - motor

/*In entire lift:
2 limit switches, one bottom and one on top
1 cancoder at bottom
1 falcon to rotate string
*/

public class Lift extends SubsystemBase{
     private TalonSRX spinMotor;
     private Cancoder spinCanCoder;


    public Lift () 
    {
        spinMotor = new FalconSRX(KSpinMotorID);
    }

}