// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.hal.ThreadsJNI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightCalculationsData;

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

  public double EstimateDistanceToTarget(){
    
    //add actual height from camera lense to floor
    double limelightHeight_Inch = LimelightCalculationsData.LimelightHeight_FromLenseToGround;

    //add actual eight of target
    double heightOfTarget_Inch = LimelightCalculationsData.HeightOfTargetInches;

    //add actual angle rotation of camera exactly from vertical
    double limelightMountAngle = LimelightCalculationsData.LimelightMountAngle;

    //limelight angle from crosshair to target
    double limelightAngleToTarget_Vertical = ty.getDouble(0.0);


    double angleToTargetDegrees = limelightMountAngle + limelightAngleToTarget_Vertical;
    double angleToTargetRadians = angleToTargetDegrees * (3.14159/180);

    double distanceFromLimelightToTargetInch = (heightOfTarget_Inch - limelightHeight_Inch)/Math.tan(angleToTargetDegrees);

    return distanceFromLimelightToTargetInch;
  }

  public void getInRange_ofTarget(){
    //set kp Distance
    double kpDistance = -0.0;
    double currentDistance = EstimateDistanceToTarget();
    
    //set desired distance
    double desiredDistance = 0;
    double distance_Error = desiredDistance - currentDistance;

    double adjustDriving = kpDistance * distance_Error;
    
    //make motors on command(s)to run at the adjusted driving speed


  }
  public void getInRange_andPlace(){
    double kpAim = 0.0;
    double kpDistance = -0.0;

    //double heading_error = tx;
    double distance_error = 0.0;
    

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
