package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.Lift.*;

public class LiftUp extends Command {
    private LiftUp lift;

    public LiftUp(LiftUp lift) {
        this.lift = lift;
        //addRequirements(lift);
    }

    @Override 
    public void execute () {
        //lift.LiftUp(KSpinMotor);
    }

    @Override
    public void end(boolean interrupted) {
      //LiftUp.LiftUpStop();
  
    }
}