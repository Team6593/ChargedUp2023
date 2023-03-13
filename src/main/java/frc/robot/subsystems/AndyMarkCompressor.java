// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ser.AnyGetterWriter;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AndyMarkCompressor extends SubsystemBase {
  
  private Compressor andymarkCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  public void enableCompressor() {
    andymarkCompressor.enableDigital();
  }

  public void disableCompressor() {
    andymarkCompressor.disable();
  }

  public void displayCompressorData() {
    boolean enabled = andymarkCompressor.isEnabled();
    boolean pressureSwitch = andymarkCompressor.getPressureSwitchValue();
    double pressure = andymarkCompressor.getPressure();
    double current = andymarkCompressor.getCurrent();
    
    SmartDashboard.putBoolean("Compresser enabled: ", enabled);
    SmartDashboard.putNumber("Compressor Current", current);
    SmartDashboard.putNumber("Pressure: ", pressure);
    SmartDashboard.putBoolean("Pressure Switch: ", pressureSwitch);
  }

  /** Creates a new AndyMarkCompressor. */
  public AndyMarkCompressor() {}

  @Override
  public void periodic() {
    displayCompressorData();
  }
}
