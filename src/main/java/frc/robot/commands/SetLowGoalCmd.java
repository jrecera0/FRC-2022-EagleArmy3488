// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

/**
 * SetLowGoalCmd class that sets the shooter to its "Low Goal" speed and threshold
 */
public class SetLowGoalCmd extends CommandBase {
  private final Shooter shooter;
  /**
   * Creates a new SetLowGoalCmd.
   * @param shooter Shooter to set the speeds and thresholds of
   */
  public SetLowGoalCmd(Shooter shooter) {
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  /**
   * When scheduled, the init function will run once to run the {@code setToLowGoal()}
   * method found within {@link Shooter}
   */
  @Override
  public void initialize() {
    shooter.setToLowGoal();
    // System.out.println(this.toString() + " started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println(this.toString() + " ended!");
  }

  // Returns true when the command should end.
  /**
   * This is set to true so as to immediately finish the command once scheduled, as the task only
   * needs to be run once in {@code initialize()}
   */
  @Override
  public boolean isFinished() {
    return true;
  }
}
