// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DoubleSolenoidChannels;
import frc.robot.Utils.UnitConverter;

public class DriveTrain extends SubsystemBase {
  
  // refrence constants to get motor ID's
  public static Constants.Motors motors = new Constants.Motors();
  public static UnitConverter unitConverter = new UnitConverter();
  public static DoubleSolenoidChannels doubleSolenoidChannels = new DoubleSolenoidChannels();

  // motor controllers
  public WPI_TalonFX masterRight = new WPI_TalonFX(motors.MasterRight); // m right
  public WPI_TalonFX masterLeft = new WPI_TalonFX(motors.MasterLeft); // m left
  public WPI_TalonFX followerLeft = new WPI_TalonFX(motors.FollowerLeft); // f left
  public WPI_TalonFX followerRight = new WPI_TalonFX(motors.FollowerRight); // f right

  public final MotorControllerGroup DtLeft = new MotorControllerGroup(masterLeft, followerLeft);
  public final MotorControllerGroup DtRight = new MotorControllerGroup(masterRight, followerRight);
  
  private final DifferentialDrive Drive = new DifferentialDrive(DtLeft, DtRight);

  // Limit Switches
  // Uncomment the following lines below if there are limit switches on the robot
  //private DigitalInput dtRightTopLimitSwitch = new DigitalInput(0);
  //private DigitalInput dtRightBottomLimitSwitch = new DigitalInput(1);
  //private DigitalInput dtLeftTopLimitSwitch = new DigitalInput(2);
  //private DigitalInput dtLeftBottomLimitSwitch = new DigitalInput(3);

  private DoubleSolenoid shifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, doubleSolenoidChannels.ForwardChannelDt, doubleSolenoidChannels.ReverseChannelDt);


  /** Creates a new DriveTrain. */
  public DriveTrain() {}



  // MOTOR INIT
  public void dtInit() {
    
    followerLeft.follow(masterLeft);
    followerRight.follow(masterRight);
    
    //Ensure motor output is nuetral during initialization
    masterRight.set(0);
    masterLeft.set(0);
    followerRight.set(0);
    followerLeft.set(0);

    //setBrakeMode();
    
    final TalonFXConfiguration config = new TalonFXConfiguration(); // Creating an instance to


    // Comment this code out if the robot 'stutters' while driving
    // This limits the amount of electricity the motor can recieve,
    // but this can cause the robot to drive slow, or 'stutter'
    // because of the motor's safety measures
    
    config.supplyCurrLimit.enable = false;
    config.supplyCurrLimit.triggerThresholdCurrent = 70;
    config.supplyCurrLimit.triggerThresholdTime = 3.0;
    config.supplyCurrLimit.currentLimit = 70;
    
    masterRight.configAllSettings(config);
    masterLeft.configAllSettings(config);
    followerRight.configAllSettings(config);
    followerLeft.configAllSettings(config);

    /* 
    masterRight.set(TalonFXControlMode.PercentOutput, 0);
    masterLeft.set(TalonFXControlMode.PercentOutput, 0);
    followerRight.set(TalonFXControlMode.PercentOutput, 0);
    followerLeft.set(TalonFXControlMode.PercentOutput, 0);
    */
    
    // set integrated sensor for PID, this doesn't matter even if PID isn't used
    config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;



    masterRight.setInverted(true);
    followerRight.setInverted(true);
    followerLeft.setInverted(false);
    masterLeft.setInverted(false);

    // reset all motor positions on init
    masterRight.setSelectedSensorPosition(0);
    masterLeft.setSelectedSensorPosition(0);
    followerRight.setSelectedSensorPosition(0);
    followerLeft.setSelectedSensorPosition(0);

    setBrakeMode();
  }

  // MOTORS
  public void setBrakeMode() {
    masterLeft.setNeutralMode(NeutralMode.Brake);
    masterRight.setNeutralMode(NeutralMode.Brake);
    followerLeft.setNeutralMode(NeutralMode.Brake);
    followerRight.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoastMode() {
    masterLeft.setNeutralMode(NeutralMode.Coast);
    masterRight.setNeutralMode(NeutralMode.Coast);
    followerLeft.setNeutralMode(NeutralMode.Coast);
    followerRight.setNeutralMode(NeutralMode.Coast);
  }


  public void setLeftMotorspeed(double leftmoterspeed) {
    DtLeft.set(leftmoterspeed);
  }

  public void setRightMotorspeed(double rightmotorspeed) {
    DtRight.set(rightmotorspeed);
  }


  public void stopAllMotors() {
    masterRight.stopMotor();
    masterLeft.stopMotor();
    followerRight.stopMotor();
    followerLeft.stopMotor();
  }

  // DRIVING
  public void curveDrive(double speed, double rotation, boolean turnInPlace) {
    Drive.curvatureDrive(speed, rotation, turnInPlace);
  }

  public void tankDrive(double speed, double rotation) {
    Drive.tankDrive(speed, rotation);
  }

  public void arcadeDrive(double xSpd, double zRot) {

    Drive.arcadeDrive(xSpd, zRot);
  }

  public void autonDrive(double speed) {
    DtRight.set(speed);
    DtLeft.set(speed);

  }

  
  // SOLENOID/SHIFTERS
  public void highGear(){
    shifter.set(Value.kForward);
  }

  public void lowGear(){
    shifter.set(Value.kReverse);
  }

  public void dtShifterOff(){
    shifter.set(Value.kOff);
  }


  public void drive(double motorspeed) {
    masterLeft.set(ControlMode.PercentOutput, motorspeed);
    masterRight.set(ControlMode.PercentOutput, motorspeed);
    followerLeft.set(ControlMode.PercentOutput, motorspeed);
    followerRight.set(ControlMode.PercentOutput, motorspeed);
  }

  public void driveDistance(double diameterOfWheel, double toGetDistanceInInchesMultiplier){
    double distanceToTravel = unitConverter.distanceToTravelCalcultions(diameterOfWheel, motors.FalconUnitsPerRevolution, toGetDistanceInInchesMultiplier);
    // 37156.244632 encoder units.
    // To get driving distance in inches 
    // divide inches that will want to be 
    // driven by 27.
    while(getRightSideMotorPosition() < distanceToTravel){
      drive(0.3);
    }

    // System.out.println("distance to travel " + distanceToTravel);
  }
  
  /**
   * resets all drivetrain sensor positions to 0,
   * call this method in robotInit(), teleopInit(), and autonomousInit()
   */
  public void resetAllMotorPosition() {
    masterLeft.setSelectedSensorPosition(0);
    followerLeft.setSelectedSensorPosition(0);
    masterRight.setSelectedSensorPosition(0);
    followerRight.setSelectedSensorPosition(0);
  }

  // MOTOR POSITION/SENSOR
  /**
   * this function only returns a value from the masterRight motor
   * @return rps (rotations-per-second)
   */
  public double getRotationsPerSecond(WPI_TalonFX motor) {
    double sensorVelocity = motor.getSelectedSensorVelocity();
    double rps = sensorVelocity / motors.FalconUnitsPerRevolution * 10;
    return rps;
    
  }

  /**
   * this function only returns a value from the masterRight motor
   * @return rpm (rotations-per-minute)
   */
  public double getRotationsPerMinute(WPI_TalonFX motor) {
    double sensorVelocity = motor.getSelectedSensorVelocity();
    double rpm = sensorVelocity / motors.FalconUnitsPerRevolution * 10;
    return rpm;
  }

  /**
   * this function only returns a value from the masterRight motor
   * @return rotations (sensor position / 2048)
   */
  public double getRotations(WPI_TalonFX motor) {
    double sensorPosition = motor.getSelectedSensorPosition();
    double rotations = sensorPosition / motors.FalconUnitsPerRevolution;
    return rotations;
  }

  /**
   * 
   * @return recu_masterRightSensorPosition, the sensor-position of the master-right motor in RECU's
   */
  public double getRightSideMotorPosition() {
    double recu_masterRightSensorPosition = unitConverter.toReadableEncoderUnit(masterRight.getSelectedSensorPosition());
    return recu_masterRightSensorPosition;
  }

  /**
   * 
   * @return recu_masterLeftSensorPosition, the sensor-position of the master-left motor in RECU's
   */
  public double getLeftSideMotorPosition() {
    double recu_masterLeftSensorPosition = unitConverter.toReadableEncoderUnit(masterLeft.getSelectedSensorPosition());
    return recu_masterLeftSensorPosition;
  }

  /**
   * displays motor position and velocity data to SmartDashboard,
   * this is meant to be called in periodic()
   */
  public void displayTalonData() {
    
    // Sensor position
    double masterRightSensorPosition = unitConverter.toReadableEncoderUnit(masterRight.getSelectedSensorPosition() );
    double slaveRightSensorPosition = unitConverter.toReadableEncoderUnit(followerRight.getSelectedSensorPosition() );
    double masterLeftSensorPosition = unitConverter.toReadableEncoderUnit(masterLeft.getSelectedSensorPosition() );
    double slaveLeftSensorPosition = unitConverter.toReadableEncoderUnit(followerLeft.getSelectedSensorPosition() );
    
    SmartDashboard.putNumber("Master Right Sensor Position", masterRightSensorPosition);
    SmartDashboard.putNumber("Follower Right Sensor Position", slaveRightSensorPosition);
    SmartDashboard.putNumber("Master Left Sensor Position", masterLeftSensorPosition);
    SmartDashboard.putNumber("Follower Left Sensor Position", slaveLeftSensorPosition);

    // Sensor velocity
    double masterRightSensorVelocity = masterRight.getSelectedSensorVelocity();
    double masterLeftSensorVelocity = masterLeft.getSelectedSensorVelocity();
    double followerLeftSensorVelocity = followerLeft.getSelectedSensorVelocity();
    double followerRightSensorVelocity = followerRight.getSelectedSensorVelocity();

    SmartDashboard.putNumber("Master Right Sensor Velocity", masterRightSensorVelocity);
    SmartDashboard.putNumber("Follower Right Sensor Velocity", followerRightSensorVelocity);
    SmartDashboard.putNumber("Master Left Sensor Velocity", masterLeftSensorVelocity);
    SmartDashboard.putNumber("Follower Left Sensor Velocity", followerLeftSensorVelocity);

    // Stator current
    double masterRightStatorCurrent = masterRight.getStatorCurrent();
    double masterLeftStatorCurrent = masterLeft.getStatorCurrent();
    double followerRightStatorCurrent = followerRight.getStatorCurrent();
    double followerLeftStatorCurrent = followerLeft.getStatorCurrent();
    
    SmartDashboard.putNumber("Master Right Stator Current", masterRightStatorCurrent);
    SmartDashboard.putNumber("Master Left Stator Current", masterLeftStatorCurrent);
    SmartDashboard.putNumber("Follower Right Stator Current", followerRightStatorCurrent);
    SmartDashboard.putNumber("Follower Left Stator Current", followerLeftStatorCurrent);
  }


   /**
    * displays TalonFX sensor data to rioLog, this method should be called in periodic()
    */
  public void printTalonData() {
    System.out.println("Sensor position, master right" + masterRight.getSelectedSensorPosition());
    System.out.println("Sensor position, slave right" + followerRight.getSelectedSensorPosition());
    System.out.println("Sensor position, master left" + masterLeft.getSelectedSensorPosition());
    System.out.println("Sensor position, slave left" + followerLeft.getSelectedSensorPosition());

    System.out.println("Sensor velocity, master right" + masterRight.getSelectedSensorVelocity());
    System.out.println("Sensor velocity, master left" + masterLeft.getSelectedSensorVelocity());
    System.out.println("Sensor velocity, slave right" + followerRight.getSelectedSensorVelocity());
    System.out.println("Sensor velocity, slave left" + followerLeft.getSelectedSensorVelocity());

    System.out.println("Motor output, Master right" + masterRight.getMotorOutputPercent());
    System.out.println("Motor output, Slave right" + followerRight.getMotorOutputPercent());
    System.out.println("Motor output, Master left" + masterLeft.getMotorOutputPercent());
    System.out.println("Motor output, Slave left" + followerLeft.getMotorOutputPercent());

    System.out.println("Stator current, Master Right" + masterRight.getStatorCurrent());
    System.out.println("Stator current, Slave Right" + followerRight.getStatorCurrent());
    System.out.println("Stator Current, Master Left" + masterLeft.getStatorCurrent());
    System.out.println("Stator Current, Slave Left" + followerLeft.getStatorCurrent());

    masterRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 350);
    masterLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 350);
    followerRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 350);
    followerLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 350);
  }

  @Override
  public void periodic() {

    displayTalonData();
  }
}
