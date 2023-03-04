// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DoubleSolenoidChannels;
import frc.robot.Constants.LimitSwitchesPorts;
import frc.robot.Constants.Motors;
import frc.robot.Constants.PIDValues;
import frc.robot.Utils.UnitConverter;

public class Arm extends SubsystemBase {

  // Instances of classes
  private Motors motors = new Motors();
  private UnitConverter unitConverter = new UnitConverter();
  private DoubleSolenoidChannels doubleSolenoidChannels = new DoubleSolenoidChannels();
  private PIDValues pidValues = new PIDValues();


  //Motor/s
//  private WPI_TalonFX armMotorUpDown = new WPI_TalonFX(motors.ArmMotorUpDownID);
//  private WPI_TalonFX armMotorLeft = new WPI_TalonFX(motors.ArmMotorLeftID);
//  private WPI_TalonFX armMotorRight = new WPI_TalonFX(motors.ArmMotorRightID);

  //solenoids
  private DoubleSolenoid armSolenoidCloseAndOpen = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, doubleSolenoidChannels.ArmCloseChannel, doubleSolenoidChannels.ArmOpenChannel);
  private DoubleSolenoid armExtandAndRetract = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, doubleSolenoidChannels.ArmExtendChannel, doubleSolenoidChannels.armRetractChannel);

  // //limit switches (WIP)
  // private DigitalInput armLimitSwitchOne = new DigitalInput(LimitSwitchesPorts.ArmLimitSwitchPortOne);
  // private DigitalInput armLimitSwitchTwo = new DigitalInput(LimitSwitchesPorts.ArmLimitSwitchPortTwo);

  /* Creates a new Hand. */
  public Arm() {}

  public void armInit(){
    
  }

  // // Limit switch methods (WIP)
  // public void armLimitSwitchUp(double armMotorSpeed) {
  //   if (armLimitSwitchOne.get() == true) {
  //     armMotorUpDown.set(armMotorSpeed);
  //   } else if (armLimitSwitchOne.get() == false) {
  //     armMotorUpDown.stopMotor();
  //     armBrakeMode();
  //   }
  // }

  // public void armLimitSwitchDown(double armMotorSpeed) {
  //   if (armLimitSwitchTwo.get() == true) {
  //     armMotorUpDown.set(-armMotorSpeed);
  //   } else if (armLimitSwitchTwo.get() == false) {
  //     armMotorUpDown.stopMotor();
  //     armBrakeMode();
  //   }
  // }

  public void armClose(){
    armSolenoidCloseAndOpen.set(Value.kForward);
  }

  public void armOpen(){
    armSolenoidCloseAndOpen.set(Value.kReverse);
  }

  public void armExtend(){
    armExtandAndRetract.set(Value.kForward);
  }

  public void armRetract(){
    armExtandAndRetract.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
