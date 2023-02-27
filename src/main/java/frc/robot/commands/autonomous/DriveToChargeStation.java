// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils.UnitConverter;
import frc.robot.subsystems.DriveTrain;

public class DriveToChargeStation extends CommandBase {
  
  UnitConverter unitConverter = new UnitConverter();
  public DriveTrain driveTrain;
  public double encoderDistance;

  //public double leftMotorPosition = driveTrain.getLeftSideMotorPosition();
  

  /** Creates a new DriveToChargeStation. */
  public DriveToChargeStation(DriveTrain driveTrain, double encoderDistance) {
    this.driveTrain = driveTrain;
    this.encoderDistance = encoderDistance;
    // Use addRequirements() here to declare subsystem dependencies.
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
    double rightMotorPosition = driveTrain.masterRight.getSelectedSensorPosition();
    double rightMotorPosition_RECU = unitConverter.toReadableEncoderUnit(rightMotorPosition);

    // This works off of sensor ticks and not a unit of measurement,
    // like feet, meters, yards, inch etc. To find out the amount of rotations
    // needed to drive to the Charge Station (docked, not yet engaged) then
    // enable the robot in FRC Driver Station and push the robot until you
    // have reached the Charge Station. Then check the motor position, 
    // will be displayed in SmartDashboard. The numnber displayed is 
    // already converted into a RECU, no conversion needed.

    // drive until we reach the set motor position
    if (rightMotorPosition_RECU < encoderDistance) {
      driveTrain.drive(.2);
    } else {
      driveTrain.stopAllMotors();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setBrakeMode();
    driveTrain.stopAllMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
