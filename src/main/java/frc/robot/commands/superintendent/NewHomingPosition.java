// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superintendent;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Reeler;

public class NewHomingPosition extends CommandBase {
  
  Elevator elevator;
  Reeler reeler;
  Arm arm;

  // THIS CODE DOESNT WORK PLEASE DO NOT EVER USE EVER
  boolean bottomLimitSwitchPressed = elevator.minHeightLimitSwitch.get();
  boolean armBottomLimitSwitchPressed = arm.armLimitSwitchBottom.get();
  
  /** Creates a new HomingPosition. */
  public NewHomingPosition(Elevator elevator, Reeler reeler, Arm arm) {
    this.reeler = reeler;
    this.elevator = elevator;
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    addRequirements(reeler);
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    reeler.reelerInit();
    arm.armInit();
    elevator.elevatorInit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!bottomLimitSwitchPressed && !armBottomLimitSwitchPressed) {
      elevator.elevate(-.05);
      reeler.reelArmUp(-.05);
      arm.rotateDownwards(.05);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //elevator.elevatorStop();
    //reeler.stopReelerMotor();
    //arm.stopArmMotor();

    elevator.elevatorBrake();
    reeler.reelerBrakeMotor();
    arm.armBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
