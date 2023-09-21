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
import frc.robot.commands.arm.ArmBrake;
import frc.robot.commands.arm.ArmClose;
import frc.robot.commands.arm.ArmDown;
import frc.robot.commands.arm.ArmOpen;


import frc.robot.commands.ElevatorCommands.RestingPosition;
import frc.robot.commands.ElevatorCommands.StartingConfig;
import frc.robot.commands.DoNothing;
import frc.robot.commands.ElevatorCommands.ElevatorDown;
import frc.robot.commands.ElevatorCommands.ElevatorStop;
import frc.robot.commands.ElevatorCommands.ElevatorUp;
import frc.robot.commands.RefactoredCommands.AdjustElevatorDown;
import frc.robot.commands.RefactoredCommands.AdjustElevatorUp;
import frc.robot.commands.RefactoredCommands.AdjustReelerDown;
import frc.robot.commands.RefactoredCommands.AdjustReelerUp;
import frc.robot.commands.RefactoredCommands.AutonomousScoring;
import frc.robot.commands.RefactoredCommands.ConeSecure;
import frc.robot.commands.RefactoredCommands.EmergencyStopCommand;
import frc.robot.commands.RefactoredCommands.HomingPosition;
import frc.robot.commands.RefactoredCommands.ReelAndRotateUp;
import frc.robot.commands.RefactoredCommands.HumanStation;
import frc.robot.commands.RefactoredCommands.SoftExit;
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
import frc.robot.commands.autonomous.TaxiWithGyro;
import frc.robot.commands.drivetrain.DriveTrain_DefaultCommnad;
import frc.robot.commands.drivetrain.HighGear;
import frc.robot.commands.drivetrain.LowGear;
import frc.robot.commands.reeler.ReelArmDown;
import frc.robot.commands.reeler.ReelArmUp;
import frc.robot.subsystems.AndyMarkCompressor;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Reeler;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraStream;
import frc.robot.subsystems.vision.LimeLight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
  public final AndyMarkCompressor compressor;
  public final Arm arm;  
  public final Reeler reeler;
  
  // Make sure this is public so you can call camInit()
  CameraStream camera;
  Camera aprilTagCamera;
  public final LimeLight limeLight;
  public final NavX navX;


  private Constants constants = new Constants();
  private ButtonBoard buttonBoardButtons = new ButtonBoard();
  private xBox xbox = new xBox();
  private SpeedsForMotors speedsForMotors = new SpeedsForMotors();
  private Autonomous autonomous = new Autonomous();
  //IO
  private final XboxController xboxController = new XboxController(constants.XboxController_Port);
  private final Joystick joystick = new Joystick(2);

  private JoystickButton leftThumbButton, rightThumbButton;


  //Buttons for xbox controller
  private JoystickButton rightTrigger, leftTrigger, aButton, xButton, 
                         yButton, bButton, rightClick, leftClick,
                         menuButton, windowButton;


  
  private Joystick buttonBoard = new Joystick(constants.ButtonBoard_Port);
  private JoystickButton armExtendButton, armRetractButton, startingConfigButton, coneSecureButton,
                         floorPickupButton, grabButton, releaseButton, humanStationButton, 
                         adjustReelerDown, adjustReelerUp, homingButton;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {    

    //initialize subystems
    navX = new NavX();
    limeLight = new LimeLight();
    driveTrain = new DriveTrain();
    arm = new Arm();
    reeler = new Reeler();
    camera = new CameraStream();
    aprilTagCamera = new Camera();
    elevator = new Elevator();
    compressor = new AndyMarkCompressor();

    driveTrain.setDefaultCommand(new DriveTrain_DefaultCommnad(driveTrain, xboxController, joystick));

    // this method polls buttons every 'tick'
    // and handles button->command
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    leftThumbButton = new JoystickButton(joystick, 3);
    leftThumbButton.onTrue(new HumanStation(reeler, elevator, arm));
    rightThumbButton = new JoystickButton(joystick, 4);
    rightThumbButton.onTrue(new HomingPosition(reeler, elevator, arm));

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
    armExtendButton = new JoystickButton(buttonBoard, buttonBoardButtons.ArmExtend);
    armRetractButton = new JoystickButton(buttonBoard, buttonBoardButtons.ArmRetract);
    startingConfigButton = new JoystickButton(buttonBoard, buttonBoardButtons.StartingConfig);
    homingButton = new JoystickButton(buttonBoard, buttonBoardButtons.Homing);
    //floorPickupButton = new JoystickButton(buttonBoard, buttonBoardButtons.FloorPickup);
    grabButton = new JoystickButton(buttonBoard, buttonBoardButtons.grab);
    releaseButton = new JoystickButton(buttonBoard, buttonBoardButtons.release);
    //humanStationButton = new JoystickButton(buttonBoard, buttonBoardButtons.HumanStation);
    //coneSecureButton = new JoystickButton(buttonBoard, buttonBoardButtons.ConeSecure);

    adjustReelerUp = new JoystickButton(buttonBoard, buttonBoardButtons.AdjustReelerUp);
    adjustReelerDown = new JoystickButton(buttonBoard, buttonBoardButtons.AdjustReelerDown);
    
    adjustReelerDown.whileTrue(new AdjustReelerDown(reeler));
    adjustReelerUp.whileTrue(new AdjustReelerUp(reeler));
    // button -> command handling
    // button board bindings

    // rewrite all the commands being used here
    //elevatorUpButton.onTrue(new ArmBrake(arm).andThen(new ElevatorUp(elevator, -.1)) ); // NEW
    startingConfigButton.whileTrue(new ElevatorDown(elevator, 0.25));
    homingButton.whileTrue(new ElevatorUp(elevator, 0.25));
    // DNR
    //floorPickupButton.onTrue(new HomingPosition(reeler, elevator, arm)); // DNR

    //coneSecureButton.onTrue(new ConeSecure(reeler, elevator, arm));

    // TODO: fix button binding
    // ArmOpen extends
    // ArmClose retracts
    // ArmRetract grabs
    // Arm Extend releases
    // NOTE: i fixed the sol channels in Constants - MQ
    armExtendButton.onTrue(new ArmExtend(arm)); // DNR, WORKS
    armRetractButton.onTrue(new ArmRetract(arm)); // DNR, WORKS
    releaseButton.onTrue(new ArmOpen(arm)); // DNR, WORKS
    grabButton.onTrue(new ArmClose(arm)); // DNR, WORKS

    // What's this even for?

    //armAndReelerUpButton.whileTrue(new ReelAndRotateUp(arm, reeler)); // NEW,change later ArmDown->ReelerAndElevatorUp
    //scoringMidButton.onTrue(new ArmDown(arm, reeler)); // NEW

    //humanStationButton.onTrue(new HumanStation(reeler, elevator, arm));
    //rightTrigger.whileTrue(new AdjustElevatorUp(elevator));
    //leftTrigger.whileTrue(new AdjustElevatorDown(elevator));

    aButton.onTrue(new HighGear(driveTrain));
    xButton.onTrue(new LowGear(driveTrain));

    // disable.onTrue(new EmergencyStopCommand(driveTrain, elevator, arm, reeler));
    //emergencyStop.onTrue(new SoftExit());

    // xbox button bindings
    //aButton.onTrue(new ArmDownGrab(arm, speedsForMotors.ArmSpeed, reeler, speedsForMotors.ReelerSpeed));
    //bButton.onTrue(new StopArmAndReeler(arm, reeler));
    // aButton.onTrue(new ElevatorDownCommand(elevator, speedsForMotors.elevator_setSpeed));
    // yButton.onTrue(new ElevatorUpCommand(elevator, speedsForMotors.elevator_setSpeed));
    // xButton.onTrue(new ElevatorStopCommand(elevator));
    // You may have to adjust these values
    //yButton.whileTrue(new ReelArmUp(reeler, .3));
    //aButton.whileTrue(new ReelArmDown(reeler, .3));

    // yButton.whileTrue(new ReelArmUp(reeler));
    //aButton.onTrue(new ArmDownGrab(arm, SpeedsForMotors.ArmSpeed, reeler, SpeedsForMotors.ReelerSpeed));
    //bButton.onTrue(new StopArmAndReeler(arm, reeler));

    // aButton.onTrue(new DriveDistanceUsingCalculations(driveTrain, 5.775, 2.5));
    // xButton.onTrue(new DriveTrainStop(driveTrain));
    //aButton.onTrue(new ArmRetract(arm));
    //aButton.onTrue(new LowGear(driveTrain));
    //xButton.onTrue(new ArmExtend(arm));
    //yButton.onTrue(new ConeSecure(reeler, elevator, arm));
    // yButton.onTrue(new HomingPosition(reeler, elevator, arm)
    //   .andThen(new HumanStation(reeler, elevator, arm) )
    //   .andThen(new ArmExtend(arm))
    // );
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return new DoNothing();
    //return new AutonomousScoring(reeler, elevator, arm);
    //.andThen(new DriveToChargeStation(driveTrain, autonomous.encoderDistanceToChargeStation))
    //.andThen(new BalanceOnChargeStation(driveTrain, navX));
    
     //return new DriveToChargeStation(driveTrain, autonomous.encoderDistanceToChargeStation);
     //.andThen(new BalanceOnChargeStation(driveTrain, navX));

    // IF THE ABOVE AUTON COMMAND DOESN'T WORK USE THE OLD COMMAND HERE:
    return new TaxiWithGyro(driveTrain, .16); 
    // taxi backwards for 5 seconds then stop
    // might have to invert motorspeed to a negative
     
  }
} 
