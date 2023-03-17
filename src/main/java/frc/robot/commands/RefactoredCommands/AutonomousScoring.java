// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RefactoredCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmClose;
import frc.robot.commands.arm.ArmExtend;
import frc.robot.commands.arm.ArmOpen;
import frc.robot.commands.autonomous.AutoHomingPosition;
import frc.robot.commands.autonomous.AutoHumanStation;
import frc.robot.commands.autonomous.AutoStartingConfig;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Reeler;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousScoring extends SequentialCommandGroup {
  /** Creates a new AutonomousScoring. */
  private Arm arm; 
  private Elevator elevator;
  private Reeler reeler;
  public AutonomousScoring(Reeler reeler, Elevator elevator, Arm arm) {
    this.arm = arm;
    this.reeler = reeler;
    this.elevator = elevator;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new AutoStartingConfig(elevator, reeler, arm),
      new AutoHomingPosition(reeler, elevator, arm),
      new AutoHumanStation(reeler, elevator, arm),
      new ArmOpen(arm).withTimeout(1),
      new ArmExtend(arm)
      );
  }
}
