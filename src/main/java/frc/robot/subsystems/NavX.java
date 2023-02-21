// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavX extends SubsystemBase {
  
  public AHRS navX = new AHRS(SPI.Port.kMXP);

  /** Creates a new NavX. */
  public NavX() {

  }

  // Helper methods
  public void reset() {
    navX.reset();
  }
  public void calibrate() {
    navX.calibrate();
  }

  public double getAngle() {
    return navX.getAngle();
  }

  public float getYaw() {
    return navX.getYaw();
  }

  public float getAltitude() {
   return navX.getAltitude();
  }

  public float getVelocityX() {
    return navX.getVelocityX();
  }

  public float getVelocityY() {
    return navX.getVelocityY();
  }

  public float getVelocityZ() {
    return navX.getVelocityZ();
  }

  // SmartDashboard methods
  public void displayNavXData() {
    SmartDashboard.putNumber("X Velocity", getVelocityX());
    SmartDashboard.putNumber("Y velocity", getVelocityY());
    SmartDashboard.putNumber("Z Velocity", getVelocityZ());

    SmartDashboard.putNumber("Yaw", getYaw());
    SmartDashboard.putNumber("Altitude", getAltitude());
    SmartDashboard.putNumber("Angle", getAngle());
  }


  @Override
  public void periodic() {
    displayNavXData();
  }
}
