// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.UnitConverter;

public class LimeLight extends SubsystemBase {

  // Use the limelight finder tool to change the limelight name
  // if you change the name of the limelight, modify the string arg in .getTable();
  // to match the name of the limelight
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry validTargets = table.getEntry("tv"); // 0 or 1
  NetworkTableEntry tx = table.getEntry("tx"); // horizontal offset from crosshair to target (-27-27)
  NetworkTableEntry ty = table.getEntry("ty"); // Vertical offset from crosshair to target (-20.5-20.5)
  NetworkTableEntry targetArea = table.getEntry("ta");

  /** Creates a new LimeLight. */
  public LimeLight() {}
  
  UnitConverter unitConverter = new UnitConverter();

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
  
  public double estimateDistance() {
    // these vars are defined in the top of this class
    // so I am redifining them here with an underscore
    NetworkTable _table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry _ty = _table.getEntry("ty");
    double targetOffsetAngle_Vertical = _ty.getDouble(0.0);

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 0; // limelight is upside-down

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 6.0;

    // distance from the target to the floor
    double goalHeightInches = 57; // change to (12*3) + 10

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
    System.out.println(distanceFromLimelightToGoalInches);
    return distanceFromLimelightToGoalInches;
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
    NetworkTable _table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = _table.getEntry("tv"); // 0 or 1
    double target = tv.getDouble(0.0);
    if(target == 0.0) {
      return false;
    } else {
      return true;
    }
  }
  
  @Override
  public void periodic() {
    // function testing
    displayValues();
  }
}
