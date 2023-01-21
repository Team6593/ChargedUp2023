// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TaxiWithGyro extends CommandBase {
  
  private DriveTrain driveTrain;
  double motorSpeed;
  Timer timer = new Timer();
  double startTime;

  /** Creates a new TaxiWithGyro. */
  public TaxiWithGyro(DriveTrain driveTrain, double motorSpeed) {

    this.driveTrain = driveTrain;
    this.motorSpeed = motorSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    driveTrain.dtInit();
    driveTrain.resetGyro();
    //timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double time = Timer.getFPGATimestamp();
    if (timer.getFPGATimestamp() - startTime < 5) {
      System.out.println(timer.getFPGATimestamp());
      // negative if backwards
      // positive if forwards
      driveTrain.autonDrive(-motorSpeed);
    } else {
      System.out.println(timer.getFPGATimestamp());
      driveTrain.stopAllMotors();
      System.out.println(" auton stopped");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    driveTrain.stopAllMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
