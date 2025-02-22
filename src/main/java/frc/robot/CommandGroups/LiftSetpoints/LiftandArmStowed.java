// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.LiftSetpoints;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Arm.TiltArmToSetPosition;
import frc.robot.commands.Lift.MoveLiftToPos;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Lift;

import static frc.robot.Constants.LiftConstants.LiftPositionConstants.*;
import static frc.robot.Constants.ArmConstants.ArmPositionConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LiftandArmStowed extends ParallelCommandGroup {
  /** Creates a new LiftandArmStowed. */
  public LiftandArmStowed(Arm arm, Lift lift) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TiltArmToSetPosition(arm, KArmPositionStow),
      new MoveLiftToPos(lift, KLiftPositionStow)
    );
  }
}
