// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armReelerElevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmRetract;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Reeler;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmRetractAndUP extends SequentialCommandGroup {
  private Arm arm;
  private Reeler reeler;
  private double reelerSpeed;
  private double armSpeed;

  /** Creates a new ArmCloseRetractUP. */
  public ArmRetractAndUP(Arm arm, double armSpeed, Reeler reeler, double reelerSpeed) {
    this.arm = arm;
    this.reeler = reeler;
    this.armSpeed = armSpeed;
    this.reelerSpeed = reelerSpeed;

    //addRequirements(arm, reeler);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmRetract(arm),
      new ArmAndReelerUp(arm, armSpeed, reeler, reelerSpeed)
    );
  }
}
