// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;
import frc.robot.Constants.SolenoidChannels;
import frc.robot.Utils.UnitConverter;

public class Arm extends SubsystemBase {
  
  /** Creates a new Arm. */
  public Arm() {}

  private Motors motors = new Motors();
  private UnitConverter unitConverter = new UnitConverter();
  private SolenoidChannels channels = new SolenoidChannels();

  public WPI_TalonFX armMotor = new WPI_TalonFX(motors.ArmID);
  public DoubleSolenoid armSolenoid = new DoubleSolenoid(
  PneumaticsModuleType.CTREPCM, channels.armForwardChannel, channels.armReverseChannel);

  public void armBrakeMode() {
    armMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void armCoastMode() {
    armMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void resetArmEncoderPosition() {
    armMotor.setSelectedSensorPosition(0);
  }

  /**
   * moves the arm up, as if it were a curl up
   * @param motorspeed
   */
  public void curlUp(double motorspeed) {
    armMotor.set(ControlMode.PercentOutput, motorspeed);
  }

  /**
   * 'releases' the arm; moves the arm back down.
   * @param motorspeed
   */
  public void curlDown(double motorspeed) {
    armMotor.set(ControlMode.PercentOutput, -motorspeed);
  }

  public void open() {
    armSolenoid.set(Value.kForward);
  }

  public void close() {
    armSolenoid.set(Value.kReverse);
  }

  public void armInit() {
    resetArmEncoderPosition();
    armMotor.set(ControlMode.PercentOutput, 0);
    final TalonFXConfiguration armConfig = new TalonFXConfiguration();

    armConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
  }

  public void displayArmData() {
    SmartDashboard.putNumber("Arm position ", armMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm velocity ", armMotor.getSelectedSensorVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
