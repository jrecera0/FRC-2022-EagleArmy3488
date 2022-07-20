// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

/**
 * IndexCmd class that allows for power cells to be indexed and moved from the pickup
 * to the shooter.
 */
public class IndexCmd extends CommandBase {
  private final Indexer indexer;
  /**
   * Creates a new IndexCmd.
   * @param indexer Indexer that will roll the power cells to the shooter
   */
  public IndexCmd(Indexer indexer) {
    this.indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(indexer); // Removing so we can use with autoshoot if needed
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println(this.toString() + " started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * When scheduled, the indexer will be told to indexer via the {@code index()} method
   * until told to stop
   */
  @Override
  public void execute() {
    indexer.index();
  }

  // Called once the command ends or is interrupted.
  /**
   * Immediately stops the indexer once unscheduled or interrupted
   */
  @Override
  public void end(boolean interrupted) {
    indexer.stop();
    // System.out.println(this.toString() + " ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
