// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.RefactoredCommands.HumanStation;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Reeler;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class raiseElevator extends ParallelCommandGroup {

  double startingPost;
  /** Creates a new raiseElevator. */
  public raiseElevator(Reeler reeler, Elevator elevator, Arm arm) {

    startingPost = reeler.getMotorPosition();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
    new RunCommand(()-> reeler.reelArmUp(0.48), reeler)
    .until(()->reeler.getMotorPosition()>startingPost + 60),
    new RunCommand(() -> elevator.elevate(-0.56), elevator).until(()->!elevator.maxHeightLimitSwitch.get())


);
  }
}
