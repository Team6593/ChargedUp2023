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

<<<<<<< HEAD
  /** Creates a new NavX. */
=======
  /** Creates a new NavX. 
   *  On the creation of the constructor, an attempt to create a AHRS object will be made.
   *  If the NavX-kMXP can't be found, an exception will be thrown
  */
>>>>>>> 9522ea312cfb548dc03d39ff66082f7eb420144b
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

<<<<<<< HEAD
=======
  /**
   * depending on the orientation of the NavX, the pitch may actually be the roll instead
   * this occurs beacuse the RoboRIO is installed onto the robot sideways, so the roll and
   * the pitch are swapped, regardless the implemenetation of getPitch() and getRoll() will
   * not change.
   * @return pitch in degrees
   */
>>>>>>> 9522ea312cfb548dc03d39ff66082f7eb420144b
  public float getPitch() {
    return navX.getPitch();
  }

<<<<<<< HEAD
=======
  /**
   * getAngle() and getYaw() return the same number (the yaw in degrees), 
   * however, getAngle() will return a more precise measurement because
   * it returns a double precision number.
   * @return The current total accumulated yaw angle (Z axis) of the robot in degrees. 
   * This heading is based on integration of the returned rate from the Z-axis (yaw) gyro.
   */
>>>>>>> 9522ea312cfb548dc03d39ff66082f7eb420144b
  public double getAngle() {
    return navX.getAngle();
  }

<<<<<<< HEAD
=======
  /**
   * getAngle() and getYaw() return the same number (the yaw in degrees), 
   * however, getAngle() will return a more precise measurement because
   * it returns a double precision number.
   * @return the yaw in degrees
   */
>>>>>>> 9522ea312cfb548dc03d39ff66082f7eb420144b
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

<<<<<<< HEAD
=======
  /**
   * depending on the orientation of the NavX, the pitch may actually be the roll instead
   * this occurs beacuse the RoboRIO is installed onto the robot sideways, so the roll and
   * the pitch are swapped, regardless the implemenetation of getPitch() and getRoll() will
   * not change.
   * 
   * @return roll in degrees
   */
>>>>>>> 9522ea312cfb548dc03d39ff66082f7eb420144b
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
<<<<<<< HEAD
    
=======
>>>>>>> 9522ea312cfb548dc03d39ff66082f7eb420144b
  }

  @Override
  public void periodic() {
    //displayNavXData();
  }
}
