// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * SetToClimbSpeedCmd class that sets the climber speed for climbing the robot
 */
public class SetToClimbSpeedCmd extends CommandBase {
  private final Climber climber;

  /**
   * Creates a new SetToClimbSpeedCmd.
   * @param climber Climber to set the speed of
   */
  public SetToClimbSpeedCmd(Climber climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  /**
   * Simply sets the climber to its climbing speed as specified in {@link Climber} within
   * the {@code setToClimb()} method
   */
  @Override
  public void initialize() {
    climber.setToClimb();
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
