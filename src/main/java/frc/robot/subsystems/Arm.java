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
import frc.robot.Utils.UnitConverter;

public class Arm extends SubsystemBase {

  private Motors motors = new Motors();
  private UnitConverter unitConverter = new UnitConverter();
  private DoubleSolenoidChannels doubleSolenoidChannels = new DoubleSolenoidChannels();

  //Motor/s
  private WPI_TalonFX armMotor = new WPI_TalonFX(motors.armMotorID);

  //solenoids
  private DoubleSolenoid armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, doubleSolenoidChannels.ArmForwardChannel, doubleSolenoidChannels.ArmReverseChannel);
  
  /* Creates a new Hand. */
  public Arm() {}

  public void armBrakeMode(){
    armMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void armCoastMode(){
    armMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void armInit(){
    armBrakeMode();
    armMotor.setSelectedSensorPosition(0);
    
    armMotor.setInverted(true);//might have to change later
    final TalonFXConfiguration handConfig = new TalonFXConfiguration();

    handConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;


  }

  public double readableArmSensorPosition(){
    double armSensorPos = unitConverter.toReadableEncoderUnit(armMotor.getSelectedSensorPosition());
    return armSensorPos;
  }

  public double armSensorVelocity(){
    double armSensorVel = unitConverter.toReadableEncoderUnit(armMotor.getSelectedSensorVelocity());
    return armSensorVel;
  }

  public void displayArmSensorData(){
    SmartDashboard.putNumber("Hand sensor position", readableArmSensorPosition());
    SmartDashboard.putNumber("Hand sensor velocity", armSensorVelocity());

  }

  public void armClose(){
    armSolenoid.set(Value.kForward);
  }

  public void armOpen(){
    armSolenoid.set(Value.kReverse);
  }

  public void armExtend(){
    while(readableArmSensorPosition() < 228){
      armMotor.set(0.25);
    }
  }

  public void armRetract(){
    while(readableArmSensorPosition() > 10){
      armMotor.set(-0.25);
    }
  }

  @Override
  public void periodic() {
    displayArmSensorData();
    // This method will be called once per scheduler run
  }
}
