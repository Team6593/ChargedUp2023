// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armReelerElevator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ElevatorCommands.ElevatorUp;
import frc.robot.commands.reeler.ReelArmUp;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Reeler;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorAndReelerUpCommand extends ParallelCommandGroup {

  private Elevator elevator;
  private Reeler reeler;
  private double elevatorSpeed, reelerSpeed;

  /** Creates a new ElevatorUp. */
  public ElevatorAndReelerUpCommand(Elevator elevator, Double elevatorSpeed, Reeler reeler, double reelerSpeed) {
    this.elevator = elevator;
    this.reeler = reeler;
    this.elevatorSpeed = elevatorSpeed;
    this.reelerSpeed = reelerSpeed;
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ReelArmUp(reeler, reelerSpeed), new ElevatorUp(elevator, reelerSpeed));
  }
}
