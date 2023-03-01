// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveDistanceUsingCalculations extends CommandBase {
  /** Creates a new DriveDistanceUsingCalculations. */

  private DriveTrain driveTrain;
  private double diameterOfWheel;
  private double toGetDistanceInInches;

  public DriveDistanceUsingCalculations(DriveTrain driveTrain, double diameterOfWheel, double toGetDistanceInInches) {
    this.driveTrain = driveTrain;
    this.diameterOfWheel = diameterOfWheel;
    this.toGetDistanceInInches = toGetDistanceInInches;

    addRequirements(driveTrain);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.dtInit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.driveDistance(diameterOfWheel, toGetDistanceInInches);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopAllMotors();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}