// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import static frc.robot.Constants.TunerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveWithJoysticks extends Command {
private final CommandSwerveDrivetrain commandSwerveDrivetrain;

private double fbSpeed; //Speed of the robot in the x direction (forward).
private double lrSpeed; //Speed of the robot in the Y direction (sideways).
private double rot;


  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(CommandSwerveDrivetrain commandSwerveDrivetrain) {
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    addRequirements(commandSwerveDrivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    fbSpeed = Robot.m_robotContainer.getLogiLeftYAxis();
    lrSpeed = Robot.m_robotContainer.getLogiLeftXAxis();
    rot = Robot.m_robotContainer.getLogiRightXAxis();
    commandSwerveDrivetrain.applyRequest(() -> Kdrive.withVelocityX(fbSpeed * KMaxSpeed)
    .withVelocityY(lrSpeed * KMaxSpeed)
    .withRotationalRate(rot * KMaxAngularRate));  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
