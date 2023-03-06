// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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


  //Motor/s
  private Spark armReelerMotor = new Spark(motors.ArmReelerMotorID);
  private WPI_TalonFX armMotor = new WPI_TalonFX(motors.ArmMotorID);

  //solenoids
  private DoubleSolenoid armSolenoidCloseAndOpen = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, doubleSolenoidChannels.ArmCloseChannel, doubleSolenoidChannels.ArmOpenChannel);
  private DoubleSolenoid armSolenoidExtandAndRetract = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, doubleSolenoidChannels.ArmExtendChannel, doubleSolenoidChannels.armRetractChannel);

  // //limit switches (WIP)
  private DigitalInput armLimitSwitchTop = new DigitalInput(LimitSwitchesPorts.ArmLimitSwitchTop);
  private DigitalInput armLimitSwitchBottom = new DigitalInput(LimitSwitchesPorts.ArmLimitSwitchBottom);

  /* Creates a new Hand. */
  public Arm() {}

  public void armInit(){
    armBrake();
  }

  public void armBrake(){
    armMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void stopArmMotors(){
    armMotor.stopMotor();
    armReelerMotor.stopMotor();
  }

  // // Limit switch methods (WIP)- might have to change from negative to positive values later
  public void armUp(double armMotorsSpeed) {
    if (armLimitSwitchTop.get() == true) {
      armMotor.set(armMotorsSpeed);
      armReelerMotor.set(-armMotorsSpeed);

    } else if (armLimitSwitchTop.get() == false) {
      stopArmMotors();
      armBrake();
    }
  }

  public void armDown(double armMotorsSpeed) {
    if (armLimitSwitchBottom.get() == true) {
      armMotor.set(-armMotorsSpeed);
      armReelerMotor.set(armMotorsSpeed);
    } else if (armLimitSwitchBottom.get() == false) {
      armMotor.stopMotor();
      armBrake();
    }
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
  }
}
