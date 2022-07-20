// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;

/**
 * DropPivotCmd class that drops the {@link frc.robot.subsystems.Pickup} mechanism towards the front of the robot
 */
public class DropPivotCmd extends CommandBase {
  private final Pivot pivot;

  /**
   * Creates a new DropPivotCmd. 
   * @param pivot Pivot being used to lower the pickup
   */
  public DropPivotCmd(Pivot pivot) {
    this.pivot = pivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println(this.toString() + " started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Simply executes the {@code drop()} function found in {@link Pivot}
   */
  @Override
  public void execute() {
    pivot.drop();
  }

  // Called once the command ends or is interrupted.
  /**
   * Immediately stops the pivot once unscheduled or interrupted
   */
  @Override
  public void end(boolean interrupted) {
    pivot.stop();
    // System.out.println(this.toString() + " ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
