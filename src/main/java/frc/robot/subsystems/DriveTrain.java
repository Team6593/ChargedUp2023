// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Motors;

public class DriveTrain extends SubsystemBase {
  
  // refrence constants to get motor ID's
  public static Constants.Motors motors = new Constants.Motors();

  // We might not be using Sparks so this is subject to change
  private WPI_TalonFX masterRight = new WPI_TalonFX(motors.MasterRight); // m right
  private WPI_TalonFX masterLeft = new WPI_TalonFX(motors.MasterLeft); // m left
  private WPI_TalonFX slaveLeft = new WPI_TalonFX(motors.SlaveLeft); // s left
  private WPI_TalonFX slaveRight = new WPI_TalonFX(motors.SlaveRight); // s right

  private final MotorControllerGroup DtLeft = new MotorControllerGroup(masterLeft, slaveLeft);
  private final MotorControllerGroup DtRight = new MotorControllerGroup(masterRight, slaveRight);

  private final DifferentialDrive Drive = new DifferentialDrive(DtLeft, DtRight);

  private AHRS gyro; //import kauaiLabs_NavX_FRC vendor library

  public double P = 1;// might have to change number later

  /** Creates a new DriveTrain. */
  public DriveTrain() {
  
    slaveRight.follow(masterRight);
    slaveLeft.follow(masterLeft);

    masterRight.setInverted(true);
    masterLeft.setInverted(false);
    slaveLeft.setInverted(InvertType.FollowMaster);
    slaveRight.setInverted(InvertType.FollowMaster);
    //NavX Gyro setup
    try {
        gyro = new AHRS(SPI.Port.kMXP);
      } catch (RuntimeException rex) {
        DriverStation.reportError("An error occured with NavX mxp, most likely and error with installing NavX - MansourQ" + rex.getMessage(), true);
      }
      
    }


    
    public void setLeftMotorspeed(double leftmoterspeed) {
      DtLeft.set(leftmoterspeed);
    }

    public void setRightMotorspeed(double rightmotorspeed) {
      DtRight.set(rightmotorspeed);
    }

    public void driveStraight(double motorspeed) {
      double err = -gyro.getAngle(); //target angle is zero
      double turnSpeed = P * err;
      Drive.arcadeDrive(motorspeed, turnSpeed);
    }

    public void resetGyro() {
      gyro.reset();
    }


    public void stopAllMotors() {
      DtLeft.stopMotor();
      DtRight.stopMotor();
    }

    public void curveDrive(double speed, double rotation, boolean turnInPlace) {
      Drive.curvatureDrive(speed, rotation, turnInPlace);
    }

    public void tankDrive(double speed, double rotation) {
      Drive.tankDrive(speed, rotation);
    }

    public void arcadeDrive(double xSpd, double zRot) {
      Drive.arcadeDrive(xSpd, zRot, false);
    }

    public void autonDrive(double speed) {
      DtRight.set(speed);
      DtLeft.set(speed);
    }
    
    public void dtInit() {

      //Ensure motor output is nuetral during initialization
      /* 
      masterLeft.set(0);
      masterRight.set(0);
      slaveLeft.set(0);
      slaveRight.set(0);
      */

      //Typically the right side of a drivetrain must be inverted
     
      
      final TalonFXConfiguration config = new TalonFXConfiguration(); // Creating an instance to

      config.supplyCurrLimit.enable = true;
      config.supplyCurrLimit.triggerThresholdCurrent = 40;
      config.supplyCurrLimit.triggerThresholdTime = 1.0;
      config.supplyCurrLimit.currentLimit = 30;
      
      masterRight.configAllSettings(config);
      masterLeft.configAllSettings(config);
      slaveRight.configAllSettings(config);
      slaveLeft.configAllSettings(config);
      masterRight.set(TalonFXControlMode.PercentOutput, 0.6);
      masterLeft.set(TalonFXControlMode.PercentOutput, 0.6);
      slaveRight.set(TalonFXControlMode.PercentOutput, 0.6);
      slaveLeft.set(TalonFXControlMode.PercentOutput, 0.6);
      
      // set integrated sensor for PID, this doesn't matter even if PID isn't used
      config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
  }

  /**
   * this function only returns a value from the masterRight motor
   * @return rps (rotations-per-second)
   */
  public double getRotationsPerSecond() {
    double sensorVelocity = masterRight.getSelectedSensorVelocity();
    double rps = sensorVelocity / motors.falconUnitsPerRevolution * 10;
    return rps;
  }

  /**
   * this function only returns a value from the masterRight motor
   * @return rpm (rotations-per-minute)
   */
  public double getRotationsPerMinute() {
    double sensorVelocity = masterRight.getSelectedSensorVelocity();
    double rpm = sensorVelocity / motors.falconUnitsPerRevolution * 10;
    return rpm * 60.0;
  }

  /**
   * this function only returns a value from the masterRight motor
   * @return rotations (sensor position / 2048)
   */
  public double getRotations() {
    double sensorPosition = masterRight.getSelectedSensorPosition();
    double rotations = sensorPosition / motors.falconUnitsPerRevolution;
    return rotations;
  }

  public void displayTalonData() {
    System.out.println("Sensor position, master right" + masterRight.getSelectedSensorPosition());
    System.out.println("Sensor position, slave right" + slaveRight.getSelectedSensorPosition());
    System.out.println("Sensor position, master left" + masterLeft.getSelectedSensorPosition());
    System.out.println("Sensor position, slave left" + slaveLeft.getSelectedSensorPosition());

    System.out.println("Sensor velocity, master right" + masterRight.getSelectedSensorVelocity());
    System.out.println("Sensor velocity, master left" + masterLeft.getSelectedSensorVelocity());
    System.out.println("Sensor velocity, slave right" + slaveRight.getSelectedSensorVelocity());
    System.out.println("Sensor velocity, slave left" + slaveLeft.getSelectedSensorVelocity());

    System.out.println("Motor output, Master right" + masterRight.getMotorOutputPercent());
    System.out.println("Motor output, Slave right" + slaveRight.getMotorOutputPercent());
    System.out.println("Motor output, Master left" + masterLeft.getMotorOutputPercent());
    System.out.println("Motor output, Slave left" + slaveLeft.getMotorOutputPercent());

    System.out.println("Stator current, Master Right" + masterRight.getStatorCurrent());
    System.out.println("Stator current, Slave Right" + slaveRight.getStatorCurrent());
    System.out.println("Stator Current, Master Left" + masterLeft.getStatorCurrent());
    System.out.println("Stator Current, Slave Left" + slaveLeft.getStatorCurrent());

    masterRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
    masterLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
    slaveRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
    slaveLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    displayTalonData();
  }
}
