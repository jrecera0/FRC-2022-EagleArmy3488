// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * EmptyCommand class that is... actually very useless. If memory serves me right this actually
 * already exists in WPILib so you don't even need this if you need an empty command to serve as
 * a delay within your autonomous routines or for whatver other reason.
 */
public class EmptyCommand extends CommandBase {
  /** Creates a new EmptyCommand. */
  public EmptyCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
