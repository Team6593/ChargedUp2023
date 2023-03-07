
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimitSwitchesPorts;
import frc.robot.Constants.Motors;

public class Reeler extends SubsystemBase {
  
  private Motors motors = new Motors();
  private LimitSwitchesPorts limitSwitchesPorts = new LimitSwitchesPorts();

  CANSparkMax topMotor = new CANSparkMax(motors.TopMotorID, MotorType.kBrushless);
  DigitalInput armLimitSwitchTop = new DigitalInput(limitSwitchesPorts.ArmLimitSwitchTop);
  DigitalInput armLimitSwitchBottom = new DigitalInput(limitSwitchesPorts.ArmLimitSwitchBottom);

  /** Creates a new Reeler. */
  public Reeler() {}

  public void reelerInit() {
    // ensure motors don't move during initialization

    topMotor.set(0);
    reelerBrakeMotor();
  }

  public RelativeEncoder getTopMotorEncoder() {
    return topMotor.getEncoder();
  }

  public void reelArmUp(double motorspeed) {
    // does one of these have to be negative?
    topMotor.set(motorspeed);
  }

  public void reelArmDown(double motorspeed) {
    // do one of these have to be positive?
    topMotor.set(-motorspeed);
  }

  public void stopReelerMotor() {
    topMotor.stopMotor();
  }

  public void reelerBrakeMotor() {
    topMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}