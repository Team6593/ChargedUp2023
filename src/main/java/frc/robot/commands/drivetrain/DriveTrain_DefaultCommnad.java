// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveTrain_DefaultCommnad extends CommandBase {
  /** Creates a new DriveTrain_DefultCommnad. */
  private DriveTrain driveTrain;
  private XboxController xboxController;

  public DriveTrain_DefaultCommnad(DriveTrain driveTrain, XboxController xboxController) {
    
    this.xboxController = xboxController;
    this.driveTrain = driveTrain;

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
    driveTrain.arcadeDrive(xboxController.getRawAxis(1), xboxController.getRawAxis(4));
    //driveTrain.arcadeDrive(-xboxController.getRawAxis(1), -xboxController.getRawAxis(4));
    //driveTrain.arcadeDrive(xboxController.getRawAxis(1), xboxController.getRawAxis(4)); // for inverted movement
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
