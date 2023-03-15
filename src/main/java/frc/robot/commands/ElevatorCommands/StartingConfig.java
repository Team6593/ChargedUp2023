// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Reeler;

public class StartingConfig extends CommandBase {
  /** Creates a new ElevatorDownCommand. */
  private Elevator elevator;
  private Reeler reeler;
  private Arm arm;
  Boolean armTopLimitSwitchIsPressed;

  public StartingConfig(Elevator elevator, Reeler reeler, Arm arm) {
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
    arm.armInit();
    armTopLimitSwitchIsPressed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.minHeightLimitSwitch.get() == true) {
      elevator.elevate(0.15 * 1.5);
    } else if (elevator.minHeightLimitSwitch.get() == false) {
      elevator.elevatorStop();
      elevator.elevatorBrake();
    }

    if(!armTopLimitSwitchIsPressed) {
      if (arm.armLimitSwitchTop.get() == true) {
        reeler.reelArmUp(0.15 * 1.5);
        arm.rotateUpwards(0.07 * 1.5);
      } else if (arm.armLimitSwitchTop.get() == false) {
        reeler.setMotorPosition(0); // needed for ConeSecure
        armTopLimitSwitchIsPressed = true; // this should break out of loop
        reeler.stopReelerMotor();
        reeler.reelerBrakeMotor();
        arm.stopArmMotor();
        arm.armBrake();
      }
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
