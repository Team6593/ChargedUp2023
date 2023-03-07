// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armReelerElevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmClose;
import frc.robot.commands.arm.ArmExtend;
import frc.robot.commands.arm.ArmOpen;
import frc.robot.commands.arm.ArmRetract;
import frc.robot.commands.reeler.ReelArmDown;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Reeler;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmDownGrab extends SequentialCommandGroup {
  private Reeler reeler;
  private Arm arm;
  private double reelerSpeed;
  private double armSpeed;

  /** Creates a new ArmUpExtendGrab. */
  public ArmDownGrab(Arm arm, double armSpeed, Reeler reeler, double reelerSpeed) {
    this.reeler = reeler;
    this.arm = arm;
    this.reelerSpeed = reelerSpeed;
    this.armSpeed = armSpeed;
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmAndReelerDown(arm, armSpeed, reeler, reelerSpeed),
      new ArmOpen(arm),
      new ArmExtend(arm),
      new ArmClose(arm)
    );
  }
}
