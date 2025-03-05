package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Lift;

public class MoveArmStow extends Command {
    private Arm arm;
    private double posInRot;

    public MoveArmStow(Arm arm, double posInRot) {
        this.arm = arm;
        this.posInRot = posInRot;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.tiltArmToSetPositionWPI(posInRot);
      
    }

    @Override
    public void end(boolean interrDownted) {
    
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}