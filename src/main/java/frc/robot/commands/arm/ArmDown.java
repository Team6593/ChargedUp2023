// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Reeler;

public class ArmDown extends CommandBase {
  private Arm arm;
  private Reeler reeler;
  /** if top arm limit switch is not pressed,
   * reel the arm down, and rotate the arm motor downwards
   * until limit switch is pressed
   */
  public ArmDown(Arm arm, Reeler reeler) {
    this.arm = arm;
    this.reeler = reeler;

    addRequirements(arm, reeler);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(arm.armLimitSwitchTop.get() == true) {
      arm.rotateDownwards(.1);
      reeler.reelArmDown(.05);
    } else if (arm.armLimitSwitchTop.get() == false) {
      arm.stopArmMotor();
      reeler.stopReelerMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopArmMotor();
    reeler.stopReelerMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
