// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DoubleSolenoidChannels;
import frc.robot.Constants.LimitSwitchesPorts;
import frc.robot.Constants.Motors;
import frc.robot.Constants.PIDValues;
import frc.robot.Utils.UnitConverter;

//// README:
//// THIS SUBSYSTEM IS ONLY SOLENOIDS (PISTONS)
//// THIS SUBSYSTEM CANNOT MOVE UP AND DOWN BY ITSELF
//// THAT IS THE REEELER SUBSYSTEM'S JOB
//// DON'T DECLARE ANY MOTORS HERE

public class Arm extends SubsystemBase {

  // Instances of classes
  private Motors motors = new Motors();
  private UnitConverter unitConverter = new UnitConverter();
  private DoubleSolenoidChannels doubleSolenoidChannels = new DoubleSolenoidChannels();
  private PIDValues pidValues = new PIDValues();

  private LimitSwitchesPorts ports = new LimitSwitchesPorts();
  //Motor/s
  public WPI_TalonFX armMotor = new WPI_TalonFX(motors.ArmMotorID);

  //solenoids
  private DoubleSolenoid armSolenoidCloseAndOpen = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
  doubleSolenoidChannels.ArmCloseChannel, doubleSolenoidChannels.ArmOpenChannel);
  private DoubleSolenoid armSolenoidExtandAndRetract = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
  doubleSolenoidChannels.ArmExtendChannel, doubleSolenoidChannels.ArmRetractChannel);

  // //limit switches (WIP)
  public DigitalInput armLimitSwitchTop = new DigitalInput(ports.ArmLimitSwitchTop);
  public DigitalInput armLimitSwitchBottom = new DigitalInput(ports.ArmLimitSwitchBottom);

  /* Creates a new Hand. */
  public Arm() {}

  public void displayLimitSwitchStatus() {
    boolean bottomLimitSwitchStatus = armLimitSwitchBottom.get();
    boolean topLimitSwitchStatus = armLimitSwitchTop.get();

    boolean bottomLimitSwitchIsPressed = false;
    boolean topLimitSwitchIsPressed = false;

    if(bottomLimitSwitchStatus == false) { bottomLimitSwitchIsPressed = true;} 
    else if (bottomLimitSwitchStatus == true) {bottomLimitSwitchIsPressed = false;}

    if(topLimitSwitchStatus == false) { topLimitSwitchIsPressed = true;}
    else if(topLimitSwitchStatus == true) { topLimitSwitchIsPressed = false;}

    SmartDashboard.putBoolean("Arm bottom limit switch", bottomLimitSwitchIsPressed);
    SmartDashboard.putBoolean("Arm top limit switch", topLimitSwitchIsPressed);
  }

  /**
   * ensures that the arm's motor output is 0 at init
   */
  public void armInit(){
    // ensure that motor output is zero at init
    armMotor.stopMotor();
    armMotor.set(ControlMode.PercentOutput, 0);
  }

  public void armBrake(){
    armMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void stopArmMotor(){
    armMotor.stopMotor();
  }

  public void rotateUpwards(double motorspeed) {
    armMotor.set(ControlMode.PercentOutput, -motorspeed);
  }

  public void rotateDownwards(double motorspeed) {
    armMotor.set(ControlMode.PercentOutput, motorspeed);
  }

  // // Limit switch methods (WIP)
  /**
   * This needs refactoring, DO NOT USE
   */
  public void armUp(double armMotorSpeed) {
    if (armLimitSwitchTop.get() == true) {
      armMotor.set(armMotorSpeed);
    } else if (armLimitSwitchTop.get() == false) {
      armMotor.stopMotor();
      armBrakeMode();
    }
  }

  /**
   * This needs refactoring. DO NOT USE
   * @param armMotorSpeed
   */
  public void armDown(double armMotorSpeed) {
    if (armLimitSwitchBottom.get() == true) {
      armMotor.set(-armMotorSpeed);
    } else if (armLimitSwitchBottom.get() == false) {
      armMotor.stopMotor();
      armBrakeMode();
    }
  }

  public void armBrakeMode() {
    armMotor.setNeutralMode(NeutralMode.Brake);
  }

  

  // HAND METHODS, GRABBING AND RELEASING OBJECTS

  public void armClose(){
    armSolenoidCloseAndOpen.set(Value.kForward);
  }

  public void armOpen(){
    armSolenoidCloseAndOpen.set(Value.kReverse);
  }

  // EXTENDER METHODS, MOVING ARM FORWARD AND BACKWARD
  
  public void armExtend(){
    armSolenoidExtandAndRetract.set(Value.kForward);
  }

  public void armRetract(){
    armSolenoidExtandAndRetract.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    displayLimitSwitchStatus();
  }
}
