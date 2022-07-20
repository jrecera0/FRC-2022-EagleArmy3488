// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * RunClimberCmd class that runs the climber according to its current specified speed within
 * {@link frc.robot.subsystems.Climber}
 */
public class RunClimberCmd extends CommandBase {
  private final Climber climber;

  /**
   * Creates a new RunClimberCmd.
   * @param climber Climber that is being used to either extend or climb
   */
  public RunClimberCmd(Climber climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println(this.toString() + " started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Simply executes the {@code run()} function found in {@link Climber}
   */
  @Override
  public void execute() {
    climber.run();
  }

  // Called once the command ends or is interrupted.
  /**
   * Immediately stops the climber once unscheduled or interrupted
   */
  @Override
  public void end(boolean interrupted) {
    climber.stop();
    // System.out.println(this.toString() + " ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
