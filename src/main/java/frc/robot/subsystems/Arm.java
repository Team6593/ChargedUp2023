// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DoubleSolenoidChannels;
import frc.robot.Constants.Motors;

public class Arm extends SubsystemBase {

  private Motors motors = new Motors();
  private DoubleSolenoidChannels doubleSolenoidChannels = new DoubleSolenoidChannels();
  /** Creates a new Hand. */
  private WPI_TalonFX armMotor = new WPI_TalonFX(motors.armMotorID);
  private DoubleSolenoid armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, doubleSolenoidChannels.ArmForwardChannel, doubleSolenoidChannels.ArmReverseChannel);

  public Arm() {}

  public void armBrakeMode(){
    armMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void armCoastMode(){
    armMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void armInit(){
    armBrakeMode();

    final TalonFXConfiguration handConfig = new TalonFXConfiguration();

    handConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    armMotor.setSelectedSensorPosition(0);
  }

  public double getArmSensorPosition(){
    return armMotor.getSelectedSensorPosition();
  }

  public double getArmSensorVelocity(){
    return armMotor.getSelectedSensorVelocity();
  }

  public void displayArmSensorData(){
    SmartDashboard.putNumber("Hand sensor velocity", getArmSensorVelocity());
    SmartDashboard.putNumber("Hand sensor position", getArmSensorPosition());

  }

  public void armExtend_grab(){
    armSolenoid.set(Value.kForward);
  }

  @Override
  public void periodic() {
    displayArmSensorData();
    // This method will be called once per scheduler run
  }
}
