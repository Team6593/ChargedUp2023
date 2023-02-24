// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.SpeedsForMotors;
import frc.robot.Constants.InputMap.xBox;
import frc.robot.Utils.MemoryMonitor;

import frc.robot.commands.ElevatorCommands.ElevatorDownCommand;
import frc.robot.commands.ElevatorCommands.ElevatorStopCommand;
import frc.robot.commands.ElevatorCommands.ElevatorUpCommand;
import frc.robot.commands.autonomous.DriveToChargeStation;
import frc.robot.commands.drivetrain.DriveTrain_DefaultCommnad;
import frc.robot.commands.drivetrain.HighGear;
import frc.robot.commands.drivetrain.LowGear;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.vision.CamRIO;
import frc.robot.subsystems.vision.LimeLight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveTrain driveTrain;
  public final Elevator elevator;
  // Make sure this is public so you can call camInit()
  public final CamRIO rioCamera;
  public final LimeLight limeLight;
  public final NavX navX;

  private Compressor compressor;

  //Util classes
  public final MemoryMonitor memoryMonitor;

  private Constants constants = new Constants();
  private xBox xbox = new xBox();
  private SpeedsForMotors speedsForMotors = new SpeedsForMotors();
  //IO
  private XboxController xboxController = new XboxController(constants.XboxController_Port);
  private JoystickButton rightTrigger, leftTrigger, aButton, xButton, yButton;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {    

    //instances of classes
    navX = new NavX();
    limeLight = new LimeLight();
    memoryMonitor = new MemoryMonitor();
    rioCamera = new CamRIO();
    driveTrain = new DriveTrain();

    elevator = new Elevator();

    compressor = new Compressor(PneumaticsModuleType.CTREPCM);

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
    // define JoystickButton to XboxController buttons
    aButton = new JoystickButton(xboxController, xbox.Abutton);
    xButton = new JoystickButton(xboxController, xbox.Bbutton);
    yButton = new JoystickButton(xboxController, xbox.Ybutton);
    rightTrigger = new JoystickButton(xboxController, xbox.RightTrigger);
    leftTrigger = new JoystickButton(xboxController, xbox.LeftTrigger);

    // button -> command handling
    // Elevator bindings
    aButton.onTrue(new ElevatorDownCommand(elevator, speedsForMotors.elevator_setSpeed));
    yButton.onTrue(new ElevatorUpCommand(elevator, speedsForMotors.elevator_setSpeed));
    xButton.onTrue(new ElevatorStopCommand(elevator));
    
    // DriveTrain, high and low gear bindings
    rightTrigger.onTrue(new HighGear(driveTrain));
    leftTrigger.onTrue(new LowGear(driveTrain));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // DriveToChargeStation then BalanceOnChargeStation
    return new DriveToChargeStation(driveTrain, 1223.760000);
    
    // taxi backwards for 5 seconds then stop
    // might have to invert motorspeed to a negative
    // only use this when DriveToChargeStation command does not work
    //new TaxiWithGyro(driveTrain, .2); 
  }
}
