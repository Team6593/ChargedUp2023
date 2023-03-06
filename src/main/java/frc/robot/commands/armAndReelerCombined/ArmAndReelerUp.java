// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armAndReelerCombined;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.ArmUp;
import frc.robot.commands.reeler.ReelArmUp;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Reeler;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmAndReelerUp extends ParallelCommandGroup {

  private Arm arm;
  private Reeler reeler;
  private double reelerSpeed, armSpeed;
  
  /** Creates a new ArmAndReelerUp. */
  public ArmAndReelerUp(Arm arm, double armSpeed, Reeler reeler, double reelerSpeed) {
    this.arm = arm ;
    this.reeler = reeler;
    this.armSpeed = armSpeed;
    this.reelerSpeed = reelerSpeed;

    addRequirements(arm, reeler);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ArmUp(arm, armSpeed), new ReelArmUp(reeler, reelerSpeed));

  }
}