// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.Autonomous;
import frc.robot.Constants.SpeedsForMotors;
import frc.robot.Constants.InputMap.xBox;
import frc.robot.Constants.InputMap.ButtonBoard;

import frc.robot.commands.ElevatorCommands.ElevatorDown;
import frc.robot.commands.ElevatorCommands.ElevatorStop;
import frc.robot.commands.arm.ArmClose;
import frc.robot.commands.arm.ArmExtend;
import frc.robot.commands.arm.ArmOpen;
import frc.robot.commands.arm.ArmRetract;
import frc.robot.commands.arm.ArmUp;
import frc.robot.commands.armReelerElevator.ArmAndReelerDown;
import frc.robot.commands.armReelerElevator.ArmAndReelerUp;
import frc.robot.commands.armReelerElevator.ArmDownGrab;
import frc.robot.commands.armReelerElevator.ArmRetractAndUP;
import frc.robot.commands.armReelerElevator.ElevatorAndReelerUpCommand;
import frc.robot.commands.armReelerElevator.StopArmAndReeler;
import frc.robot.commands.autonomous.BalanceOnChargeStation;
import frc.robot.commands.autonomous.DriveToChargeStation;
import frc.robot.commands.drivetrain.DriveTrain_DefaultCommnad;
import frc.robot.commands.drivetrain.HighGear;
import frc.robot.commands.drivetrain.LowGear;
import frc.robot.commands.reeler.ReelArmDown;
import frc.robot.commands.reeler.ReelArmUp;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Reeler;
import frc.robot.subsystems.vision.Camera;
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
  public final Arm arm;  
  public final Reeler reeler;

  // Make sure this is public so you can call camInit()
  public final Camera rioCamera;
  public final LimeLight limeLight;
  public final NavX navX;

  private Constants constants = new Constants();
  private ButtonBoard buttonBoardButtons = new ButtonBoard();
  private xBox xbox = new xBox();
  private SpeedsForMotors speedsForMotors = new SpeedsForMotors();
  private Autonomous autonomous = new Autonomous();
  //IO
  private XboxController xboxController = new XboxController(constants.XboxController_Port);
  private Joystick buttonBoard = new Joystick(constants.ButtonBoard_Port);

  //Buttons for xbox controller
  private JoystickButton rightTrigger, leftTrigger, aButton, xButton, 
                         yButton, bButton, rightClick, leftClick,
                         menuButton, windowButton;

  //Buttons for ButtonBoard
  private JoystickButton armExtendButton, armRetractButton, elevatorUpButton, 
                         elevatorDownButton, grabButton, releaseButton,
                         armAndReelerUpButton, armAndReelerDownButton;


  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {    

    //instances of classes
    navX = new NavX();
    limeLight = new LimeLight();
    rioCamera = new Camera();
    driveTrain = new DriveTrain();
    arm = new Arm();
    reeler = new Reeler();
    elevator = new Elevator();


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
    xButton = new JoystickButton(xboxController, xbox.Xbutton);
    yButton = new JoystickButton(xboxController, xbox.Ybutton);
    bButton = new JoystickButton(xboxController, xbox.Bbutton);
    rightClick = new JoystickButton(xboxController, xbox.RightButtonClick);
    leftClick = new JoystickButton(xboxController, xbox.LeftButtonClick);
    rightTrigger = new JoystickButton(xboxController, xbox.RightTrigger);
    leftTrigger = new JoystickButton(xboxController, xbox.LeftTrigger);

    // define joystickButton to buttonBoard buttons
    armExtendButton = new JoystickButton(buttonBoard, buttonBoardButtons.ArmExtendButton);
    armRetractButton = new JoystickButton(buttonBoard, buttonBoardButtons.ArmRetractButton);
    elevatorUpButton = new JoystickButton(buttonBoard, buttonBoardButtons.ElevatorUpButton);
    elevatorDownButton = new JoystickButton(buttonBoard, buttonBoardButtons.ElevatorDownButton);
    grabButton = new JoystickButton(buttonBoard, buttonBoardButtons.GrabButton);
    releaseButton = new JoystickButton(buttonBoard, buttonBoardButtons.ReleaseButton);
    armAndReelerUpButton = new JoystickButton(buttonBoard, buttonBoardButtons.ArmAndReelerUpButton);
    armAndReelerDownButton = new JoystickButton(buttonBoard, buttonBoardButtons.ArmAndReelerDownButton);


    
    // button -> command handling
    // button board bindings
    elevatorUpButton.onTrue(new ElevatorAndReelerUpCommand(elevator, speedsForMotors.ElevatorSpeed, reeler, speedsForMotors.ReelerSpeed));
    elevatorDownButton.onTrue(new ElevatorDown(elevator, speedsForMotors.ElevatorSpeed));
    armExtendButton.onTrue(new ArmExtend(arm));
    armRetractButton.onTrue(new ArmRetract(arm));
    grabButton.onTrue(new ArmClose(arm));
    releaseButton.onTrue(new ArmOpen(arm));
    armAndReelerUpButton.onTrue(new ArmRetractAndUP(arm, speedsForMotors.ArmSpeed, reeler, speedsForMotors.ReelerSpeed));
    armAndReelerDownButton.onTrue(new ArmDownGrab(arm, speedsForMotors.ArmSpeed, reeler, speedsForMotors.ReelerSpeed));

    // xbox button bindings
    aButton.onTrue(new ArmDownGrab(arm, SpeedsForMotors.ArmSpeed, reeler, SpeedsForMotors.ReelerSpeed));
    bButton.onTrue(new StopArmAndReeler(arm, reeler));
    // aButton.onTrue(new DriveDistanceUsingCalculations(driveTrain, 5.775, 2.5));
    // xButton.onTrue(new DriveTrainStop(driveTrain));
    leftClick.onTrue(new LowGear(driveTrain));
    rightClick.onTrue(new HighGear(driveTrain));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return new DriveToChargeStation(driveTrain, autonomous.encoderDistanceToChargeStation)
    .andThen(new BalanceOnChargeStation(
      driveTrain,
      navX,
      autonomous.balancingSpeed,
      autonomous.rollThreshholdDegrees));
    
    // IF THE ABOVE AUTON COMMAND DOESN'T WORK USE THE OLD COMMAND HERE:
    //new TaxiWithGyro(driveTrain, .2); 
    // taxi backwards for 5 seconds then stop
    // might have to invert motorspeed to a negative
     
  }
} 
