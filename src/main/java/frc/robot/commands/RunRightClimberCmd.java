// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class RunRightClimberCmd extends CommandBase {
  private final Climber climber;
  /** Creates a new ExtendLeftClimberCmd. */
  public RunRightClimberCmd(Climber climber) {
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
  @Override
  public void execute() {
    // climber.runLeftClimber();
  }

  // Called once the command ends or is interrupted.
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
