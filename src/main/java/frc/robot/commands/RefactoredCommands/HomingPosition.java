// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RefactoredCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Reeler;

public class HomingPosition extends CommandBase {
  Reeler reeler;
  Elevator elevator;
  Arm arm;
  
  /** Creates a new HomingPosition. */
  public HomingPosition(Reeler reeler, Elevator elevator, Arm arm) {
    this.reeler = reeler;
    this.elevator = elevator;
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(reeler, elevator, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.armInit();
    elevator.elevatorInit();
    reeler.reelerInit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // limit switch is not pressed if true
    if(elevator.minHeightLimitSwitch.get() == true) {
      elevator.elevate(.15 * 1.5); // going down
    } else if(elevator.minHeightLimitSwitch.get() == false) {
      elevator.elevatorStop();
    }

    if(arm.armLimitSwitchBottom.get() == true) {
      reeler.reelArmUp(-.15 * 1.25);
      arm.rotateDownwards(.07 * 1.25);
    } else if(arm.armLimitSwitchBottom.get() == false) {
      reeler.stopReelerMotor();
      arm.stopArmMotor();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.elevatorStop();
    reeler.stopReelerMotor();
    arm.stopArmMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
