// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

public class BalanceOnChargeStation extends CommandBase {
  /** Creates a new BalanceOnChargeStation. */
  private DriveTrain driveTrain;
  private NavX navX;
  private double speed;

  // what is the roll in degrees when the robot is level
  private double levelRoll;

  public BalanceOnChargeStation(DriveTrain driveTrain, NavX navX, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.navX = navX;
    this.speed = speed;

    addRequirements(driveTrain, navX);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // do not use getPitch() as it actually returns the roll
    if(navX.getRoll() < levelRoll) {
      // robot is forward facing-down position
      // drive backwards slowly
      driveTrain.drive(-.1);
    } else if (navX.getRoll() > levelRoll) {
      // robot is backwards facing-up position
      // drive forwards slowly
      driveTrain.drive(.1);
    } else if (navX.getRoll() == levelRoll) {
      driveTrain.stopAllMotors();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopAllMotors();
    //driveTrain.driveTrainBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
