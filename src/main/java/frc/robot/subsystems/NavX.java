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
  public AHRS navX;

  /** Creates a new NavX. */
  public NavX() {
    try {
      navX = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException NavXNotFoundRuntimeException) {
      DriverStation.reportError("Could not find NavX-mxp, this is most likely an installation issue" + NavXNotFoundRuntimeException.getMessage(), true);
    }
  }

  public void connectionTest() {
    if(navX.isConnected()) {
      System.out.println("NavX connected");
    } else { System.out.println("NavX is not connected");}
  }

  public void reset() {
    navX.reset();
  }

  public void calibrate() {
    navX.calibrate();
  }

  public void resetYaw() {
    navX.zeroYaw();
  } 


  /**
   * Depending on the orientation of the RoboRIO, the pitch might actually be the roll instead
   * This means that the RoboRIO was installed sideways, so the pitch and roll are swapped.
   * Regardless, the implementation of getPitch and getRoll will not be changed.
   * @return Pitch (the level of tilt backwards or forwards in degrees)
   */
  public float getPitch() {
    return navX.getPitch();
  }

  /**
   * This method is the same as getYaw, with some slight differences. However, 
   * this method is more accurate because it returns a double precision number
   * @return The current total accumulated yaw angle (Z axis) of the robot in degrees. 
   * This heading is based on integration of the returned rate from the Z-axis (yaw) gyro.
   */
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

  /**
   * Depending on the orientation of the RoboRIO, the pitch might actually be the roll instead
   * This means that the RoboRIO was installed sideways, so the pitch and roll are swapped.
   * Regardless, the implementation of getPitch and getRoll will not be changed.
   * @return Roll (the level of tilt left or right measured in degrees)
   */
  public float getRoll() {
    return navX.getRoll();
  }

  // SmartDashboard methods
  public void displayNavXData() {
    SmartDashboard.putNumber("X Velocity", getVelocityX());
    SmartDashboard.putNumber("Y velocity", getVelocityY());
    SmartDashboard.putNumber("Z Velocity", getVelocityZ());

    SmartDashboard.putNumber("Yaw", getYaw());
    SmartDashboard.putNumber("Altitude", getAltitude());
    SmartDashboard.putNumber("Angle", getAngle());
    SmartDashboard.putNumber("Pitch", getPitch());

    SmartDashboard.putNumber("Roll", getRoll());
  }

  @Override
  public void periodic() {
    //displayNavXData();
  }
}
