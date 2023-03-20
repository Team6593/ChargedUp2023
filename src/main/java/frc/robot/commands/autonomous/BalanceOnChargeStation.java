// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Autonomous;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

public class BalanceOnChargeStation extends CommandBase {
  /** Creates a new BalanceOnChargeStation. */
  private DriveTrain driveTrain;
  private NavX navX;
  private Autonomous autonomous = new Autonomous();

  // what is the roll in degrees when the robot is level
  private double levelRoll = autonomous.levelDegrees;

  public BalanceOnChargeStation(DriveTrain driveTrain, NavX navX) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.navX = navX;

    addRequirements(driveTrain, navX);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.dtInit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // do not use getPitch() as it actually returns the roll
    if(Math.floor(navX.getRoll() ) < levelRoll) {
      // robot is forward facing-down position
      // drive backwards slowly
      driveTrain.drive(-.3);
    } else if (Math.floor(navX.getRoll() ) > levelRoll) {
      // robot is backwards facing-up position
      // drive forwards slowly
      driveTrain.drive(.3);
    } else if (Math.floor(navX.getRoll() ) == levelRoll) {
      driveTrain.stopAllMotors();
      driveTrain.setBrakeMode();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopAllMotors();
    driveTrain.setBrakeMode();
    //driveTrain.driveTrainBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
