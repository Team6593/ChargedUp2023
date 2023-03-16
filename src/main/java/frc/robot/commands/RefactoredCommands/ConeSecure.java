// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RefactoredCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Reeler;

public class ConeSecure extends CommandBase {
  
  Reeler reeler;
  Elevator elevator;
  double currentReelerPosition;
  Arm arm;

  /** Creates a new ConeSecure. */
  public ConeSecure(Reeler reeler, Elevator elevator, Arm arm) {
    this.reeler = reeler;
    this.elevator = elevator;
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(reeler, elevator, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    reeler.reelerInit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentReelerPosition = reeler.getMotorPosition();

    // reel up or down based on reeler position
    /* 
    if(currentReelerPosition > 0) {
      reeler.reelArmUp(-.18);
    } else if (currentReelerPosition < 0) {
      reeler.reelArmUp(.18);
    } else if (currentReelerPosition == 0) {
      reeler.stopReelerMotor();
      reeler.reelerBrakeMotor();
    } else {
      reeler.stopReelerMotor();
      reeler.reelerBrakeMotor();
    }
    */

    // jank asf code
    if(currentReelerPosition > 0) {
      if(arm.armLimitSwitchTop.get() == true) {
        reeler.reelArmUp(-.13);
      } else if (arm.armLimitSwitchTop.get() == false) {
        reeler.stopReelerMotor();
        reeler.reelerBrakeMotor();
      }
    } else if (currentReelerPosition < 0) {
      if(arm.armLimitSwitchTop.get() == true) {
        reeler.reelArmUp(.13);
      } else if (arm.armLimitSwitchTop.get() == false) {
        reeler.stopReelerMotor();
        reeler.reelerBrakeMotor();
      }
    } else if (currentReelerPosition == 0) {
      reeler.stopReelerMotor();
      reeler.reelerBrakeMotor();
    }
    //else {reeler.reelerBrakeMotor();} // brake motor regardless ???

    if (elevator.minHeightLimitSwitch.get() == true) {
      elevator.elevate(0.20 * 1.5);
    } else if (elevator.minHeightLimitSwitch.get() == false) {
      elevator.elevatorStop();
      elevator.elevatorBrake();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    reeler.stopReelerMotor();
    reeler.reelerBrakeMotor();
    elevator.elevatorBrake();
    elevator.elevatorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
