// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Utils.MemoryMonitor;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.autonomous.DriveToChargeStation;
import frc.robot.commands.autonomous.TaxiWithGyro;
import frc.robot.commands.drivetrain.DriveTrain_DefaultCommnad;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.vision.CamRIO;
import frc.robot.subsystems.vision.LimeLight;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveTrain driveTrain;

  // Make sure this is public so you can call camInit()
  public final CamRIO rioCamera;
  public final LimeLight limeLight;

  private final ExampleCommand exampleCommand;
  private final ExampleSubsystem exampleSubsystem;

  //Util classes
  public final MemoryMonitor memoryMonitor;

  private Constants constants = new Constants();
  //IO
  private XboxController xboxController = new XboxController(constants.XboxController_Port);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    limeLight = new LimeLight();
    memoryMonitor = new MemoryMonitor();
    rioCamera = new CamRIO();
    driveTrain = new DriveTrain();
    exampleSubsystem = new ExampleSubsystem();
    exampleCommand = new ExampleCommand();
    driveTrain.setDefaultCommand(new DriveTrain_DefaultCommnad(driveTrain, xboxController));
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new DriveToChargeStation(driveTrain, 1223.760000);//new TaxiWithGyro(driveTrain, .2); 
    // taxi backwards for 5 seconds then stop
    // might have to invert motorspeed to a negative
  }
}
