// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RefactoredCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Reeler;

public class ReelerAndElevatorUp extends CommandBase {
  Reeler reeler;
  Elevator elevator;
  /** Creates a new ReelerAndElevatorUp. */
  public ReelerAndElevatorUp(Reeler reeler, Elevator elevator) {
    this.reeler = reeler;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(reeler, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    reeler.reelerInit();
    elevator.elevatorInit();
    System.out.println("ReelerAndElevatorUp init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //reeler.reelArmUp(.2);
    if (elevator.maxHeightLimitSwitch.get() == false) {
      reeler.stopReelerMotor();
      reeler.reelerBrakeMotor();
      elevator.elevatorStop();
      elevator.elevatorBrake();
    } else if (elevator.maxHeightLimitSwitch.get() == true) {
      reeler.reelArmUp(.15);
      elevator.elevate(-.15);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ReelerAndElevatorUp finished");
    reeler.stopReelerMotor();
    elevator.elevatorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
