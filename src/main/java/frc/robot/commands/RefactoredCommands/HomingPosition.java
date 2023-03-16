// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RefactoredCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Reeler;

public class HomingPosition extends CommandBase {
  Reeler reeler;
  Elevator elevator;
  Arm arm;
  boolean armLimitSwitchBottomIsPressed;
  boolean elevatorBottomLimitSwitchIsPressed;
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
    armLimitSwitchBottomIsPressed = false;
    elevatorBottomLimitSwitchIsPressed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // put the if/elif blocks in the nested if-if/elif bloack if current
    // command causes the arm to 'jitter'
    if(!elevatorBottomLimitSwitchIsPressed) {
      if (elevator.minHeightLimitSwitch.get() == true) {
        elevator.elevate(0.20 * 1.5);
      } else if (elevator.minHeightLimitSwitch.get() == false) {
        elevatorBottomLimitSwitchIsPressed = true;
        elevator.elevatorStop();
        elevator.elevatorBrake();
      }
  }

    if(!armLimitSwitchBottomIsPressed) {

      if(arm.armLimitSwitchBottom.get() == true) {
        reeler.reelArmUp(-.25 * 1.5);
        arm.rotateDownwards(.1 * 1.5);
        //elevator.elevatorStop();
        //elevator.elevatorBrake();
      } else if(arm.armLimitSwitchBottom.get() == false) {
        armLimitSwitchBottomIsPressed = true;
        reeler.stopReelerMotor();
        //elevator.elevatorStop();
        //elevator.elevatorBrake();
        //arm.rotateDownwards(0);
        arm.stopArmMotor();
        arm.armBrake();
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.elevatorStop();
    reeler.stopReelerMotor();
    arm.stopArmMotor();
    arm.armBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
