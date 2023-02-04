// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.limelight;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.vision.LimeLight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autonomous_GetInRange_Place_Command extends InstantCommand {

  private DriveTrain driveTrain;
  private LimeLight limelight;

  public Autonomous_GetInRange_Place_Command(LimeLight limelight, DriveTrain driveTrain) {

    this.limelight = limelight;
    this.driveTrain = driveTrain;

    addRequirements(limelight, driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public void getInRange_ofTarget(){
    //set kp Distance
    double kpDistance = -0.0;
    double currentDistance = 0;
    
    //set desired distance
    double desiredDistance = 0;
    double distance_Error = desiredDistance - currentDistance;

    double adjustDriving = kpDistance * distance_Error;
    
    //make motors on command(s)to run at the adjusted driving speed
    driveTrain.setLeftMotorspeed(adjustDriving);
    driveTrain.setRightMotorspeed(adjustDriving);

  }
  public void getInRange_andPlace(){
    double kpAim = 0.0;
    double kpDistance = -0.0;

    //double heading_error = tx;
    double distance_error = 0.0;
    

  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
}
