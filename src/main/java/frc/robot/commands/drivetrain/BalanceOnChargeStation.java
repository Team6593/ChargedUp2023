// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

public class BalanceOnChargeStation extends CommandBase {
  /** Creates a new BalanceOnChargeStation. */
  private DriveTrain driveTrain;
  private NavX navX;
  private double dtSpeed;

  public BalanceOnChargeStation(DriveTrain driveTrain, NavX navX, double dtSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.navX = navX;
    this.dtSpeed = dtSpeed;

    addRequirements(driveTrain, navX);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(navX.getRoll() < -1){
      driveTrain.drive(-dtSpeed);

    }else if(navX.getRoll() > 1){
      driveTrain.drive(dtSpeed);

    }else if(navX.getRoll() > -1 && navX.getRoll() < 1){
      driveTrain.stopAllMotors();
      //driveTrain.driveTrainBrake();

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

