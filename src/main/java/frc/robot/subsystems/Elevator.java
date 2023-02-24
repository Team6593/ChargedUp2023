// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimitSwitchesPorts;
import frc.robot.Constants.Motors;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private Motors motorsId = new Motors();
  private LimitSwitchesPorts limitSwitchesPorts = new LimitSwitchesPorts();
  private WPI_TalonFX elevatorMotor = new WPI_TalonFX(motorsId.ElevatorMotorID);

  DigitalInput maxHeightLimitSwitch = new DigitalInput(limitSwitchesPorts.ElevatorTopLimitSwitchPort);
  DigitalInput minHeightLimitSwitch = new DigitalInput(limitSwitchesPorts.ElevatorLowLimitSwitchPort); 

  public Elevator() {}
  /**
   * 
   --Gets the value of the digital input. Returns true if circuit is open
   maxHeightLimitSwitch.get();
   *
   */
  public void elevatorUP(double elevatorSpeed){
    // This code is unstable do not use for now
    if(maxHeightLimitSwitch.get() == true){
      elevatorMotor.set(elevatorSpeed);
      
    }else if(maxHeightLimitSwitch.get() == false){
      elevatorMotor.stopMotor();
      elevatorBrake();

    }
  }

  public void elevatorDown(double elevatorSpeed){
    if(minHeightLimitSwitch.get() == true){
      elevatorMotor.set(-elevatorSpeed);

    }else if(minHeightLimitSwitch.get() == false){
      elevatorMotor.stopMotor();
      elevatorBrake();

    }
  }

  public void elevatorStop(){
    elevatorMotor.stopMotor();
    elevatorBrake();
  }

  public void elevatorBrake() {
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void elevatorInit(){
    elevatorMotor.setSafetyEnabled(true);
    elevatorMotor.setExpiration(0.5);

  }

  @Override
  public void periodic() {
    elevatorMotor.feed();
    // This method will be called once per scheduler run
  }
}
