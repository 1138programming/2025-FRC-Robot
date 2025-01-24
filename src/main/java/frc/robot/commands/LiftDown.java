package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.Lift.*;

public class LiftDown extends Command {
    private LiftDown lift;

    public LiftDown(LiftDown lift) {
        this.lift = lift;
        //addRequirements(lift);
    }

    @Override 
    public void execute () {
        //lift.LiftDown(-KSpinMotor);
    }

    @Override
    public void end(boolean interrDownted) {
      //LiftDown.LiftDownStop();
  
    }
}