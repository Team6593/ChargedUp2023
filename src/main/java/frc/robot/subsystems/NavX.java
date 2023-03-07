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

  public void setup() {
    connectionTest();
    reset();
    calibrate();
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
   * depending on the way the RoboRIO is mounted, the getPitch() method might
   * actually return the roll instead, this is beacuse the roboRIO is installed
   * sideways. Regardless, this method will return what the NavX thinks the pitch
   * is.
   * Pitch is the measure of rotation around the X axis.
   * @return pitch in degrees (-180 to 180)
   */
  public float getPitch() {
    return navX.getPitch();
  }

  public double getAngle() {
    return navX.getAngle();
  }

  /**
   * Yaw is the measure of rotation around the Z axis
   * @return yaw in degrees (-180 to 180)
   */
  public float getYaw() {
    return navX.getYaw();
  }

  public float getAltitude() {
   return navX.getAltitude();
  }

  /**
   * This method is experimental and shouldn't be used for crucial tasks,
   * the values are known to be "noisy" and innacurate.
   * @return Current velocity in m^2 (meters squared) on the X axis
   */
  public float getVelocityX() {
    return navX.getVelocityX();
  }

  /**
   * This method is experimental and shouldn't be used for crucial tasks,
   * the values are known to be "noisy" and innacurate.
   * @return Current velocity in m^2 (meters squared) on the Y axis
   */
  public float getVelocityY() {
    return navX.getVelocityY();
  }

  /**
   * This method is experimental and shouldn't be used for crucial tasks,
   * the values are known to be "noisy" and innacurate.
   * @return Current velocity in m^2 (meters squared) on the Z axis
   */
  public float getVelocityZ() {
    return navX.getVelocityZ();
  }

  /**
   * depending on how the RoboRIO is installed, the getRoll() method might
   * actually be returning the pitch instead, this is because the RoboRIO
   * is installed sideways. Regardless the method will return what the NavX
   * thinks the roll is.
   * @return roll in degrees (-180 to 180)
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
    SmartDashboard.putNumber("Roll (NavX Pitch)", getPitch());

    SmartDashboard.putNumber("Pitch (NavX Roll)", getRoll());
    
  }

  @Override
  public void periodic() {
    //displayNavXData();
  }
}
