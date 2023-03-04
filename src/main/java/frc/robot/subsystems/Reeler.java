
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;

public class Reeler extends SubsystemBase {
  
  Motors motors = new Motors();

  CANSparkMax topMotor = new CANSparkMax(motors.TopMotorID, MotorType.kBrushless);
  WPI_TalonFX bottomMotor = new WPI_TalonFX(motors.BottomMotorID);

  /** Creates a new Reeler. */
  public Reeler() {}

  public void reelerInit() {
    // ensure motors don't move during initialization
    topMotor.set(0);
    bottomMotor.set(ControlMode.PercentOutput, 0);
  }

  public void reelArmUp() {
    // does one of these have to be negative?
    topMotor.set(.3);
    
    // bottom motor may need more speed
    bottomMotor.set(ControlMode.PercentOutput, .3);
  }

  public void reelArmDown() {
    // do one of these have to be positive?
    topMotor.set(-.3);

    // bottom motor may need more speed
    bottomMotor.set(ControlMode.PercentOutput, -.3);
  }

  public void stopReelerMotors() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  public void reelerBrakeMotors() {
    topMotor.setIdleMode(IdleMode.kBrake);
    bottomMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
