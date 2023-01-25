// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.hal.ThreadsJNI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry validTargets = table.getEntry("tv"); // 0 or 1
  NetworkTableEntry tx = table.getEntry("tx"); // horizontal offset from crosshair to target (-27-27)
  NetworkTableEntry ty = table.getEntry("ty"); // Vertical offset from crosshair to target (-20.5-20.5)
  NetworkTableEntry targetArea = table.getEntry("ta");

  /** Creates a new LimeLight. */
  public LimeLight() {}
  

  public void displayValues() {
    // read vals periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = targetArea.getDouble(0.0);

    // post to SmartDashboard
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimeLightY", y);
    SmartDashboard.putNumber("LimeLightArea", area);
  }

  /** returns horizontal offset from crosshair to target as a double.
   * This value's range is -27 to 27 degrees
   */
  public double getHorizontalOffset() {
    double hOffset = tx.getDouble(0.0);
    return hOffset;
  }

  public double getVerticalOffset() {
    double vOffset = ty.getDouble(0.0);
    return vOffset;
  }

  public double getTargetArea() {
    double ta = targetArea.getDouble(0.0);
    return ta;
  }



  public boolean isTargetFound() {
    boolean targetFound;
    double target = validTargets.getDouble(0.0);
    if(target == 0.0) {
      System.out.println("no targets found");
      targetFound = false;
      return false;
    } else {
      targetFound = true;
      return true;
    }
  }
  
  @Override
  public void periodic() {
    displayValues();
  }
}
