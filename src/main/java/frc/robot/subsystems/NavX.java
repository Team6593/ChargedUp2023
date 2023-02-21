// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavX extends SubsystemBase {
  
  public AHRS navX = new AHRS(SPI.Port.kMXP);

  /** Creates a new NavX. */
  public NavX() {

  }


  public void reset() {
    navX.reset();
  }
  public void calibrate() {
    navX.calibrate();
  }

  public void getAngle() {
    navX.getAngle();
  }

  public void getYaw() {
    navX.getYaw();
  }

  public void getAltitude() {
    navX.getAltitude();
  }

  public void getVelocityX() {
    navX.getVelocityX();
  }

  public void getVelocityY() {
    navX.getVelocityY();
  }

  public void getVelocityZ() {
    navX.getVelocityZ();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
