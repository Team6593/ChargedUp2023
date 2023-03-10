// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Reeler;

public class RestingPosition extends CommandBase {
  /** Creates a new ElevatorDownCommand. */
  private Elevator elevator;
  private Reeler reeler;
  private Arm arm;

  public RestingPosition(Elevator elevator, Reeler reeler, Arm arm) {
    this.elevator = elevator;
    this.reeler = reeler;
    this.arm = arm;

    addRequirements(elevator, reeler, arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.elevatorInit();
    reeler.reelerInit();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(elevator.minHeightLimitSwitch.get() == true) {
      elevator.elevate(.15);
    } else if(elevator.minHeightLimitSwitch.get() == false) {
      elevator.elevatorStop();
      elevator.elevatorBrake();
    }
    
    if(arm.armLimitSwitchBottom.get() == true) {
      reeler.reelArmDown(.15);
    } else if(arm.armLimitSwitchBottom.get() == false) {
      reeler.stopReelerMotor();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.elevatorStop();
    elevator.elevatorBrake();
    reeler.stopReelerMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
