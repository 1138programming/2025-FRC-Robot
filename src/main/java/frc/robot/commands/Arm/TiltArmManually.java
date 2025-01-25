package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import java.util.function.DoubleSupplier;

public class TiltArmManually extends Command {
    private final Arm arm;
    private final DoubleSupplier speedSupplier;

    public TiltArmManually(Arm arm, DoubleSupplier speed) {
        this.arm = arm;
        this.speedSupplier = speed;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        // Initialization code, if needed
    }

    @Override
    public void execute() {
        arm.tiltArmManually(speedSupplier);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop(); 
        //might need to be changed
    }

    //figure out controller for ending command
    @Override
    public boolean isFinished() {
        return false; // This command runs until interrupted

    }
}
