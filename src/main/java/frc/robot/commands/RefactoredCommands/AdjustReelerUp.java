// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RefactoredCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Reeler;

public class AdjustReelerUp extends CommandBase {
  
  Reeler reeler;

  /** Creates a new AdjustReelerDown. */
  public AdjustReelerUp(Reeler reeler) {
    this.reeler = reeler;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(reeler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    reeler.reelerInit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    reeler.reelArmUp(.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    reeler.stopReelerMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
