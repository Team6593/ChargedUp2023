// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RefactoredCommands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Reeler;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EmergencyStopCommand extends InstantCommand {
  DriveTrain driveTrain;
  Elevator elevator;
  Arm arm;
  Reeler reeler;

  public EmergencyStopCommand(DriveTrain driveTrain, Elevator elevator, Arm arm, Reeler reeler) {
    this.driveTrain = driveTrain;
    this.elevator = elevator;
    this.arm = arm;
    this.reeler = reeler;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, elevator, arm, reeler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.stopAllMotors();
    driveTrain.setBrakeMode();
    elevator.elevatorStop();
    elevator.elevatorBrake();
    arm.stopArmMotor();
    arm.armBrakeMode();
    reeler.stopReelerMotor();
    reeler.reelerBrakeMotor();
    CommandScheduler.getInstance().cancelAll();
  }
}
