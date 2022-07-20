// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

/**
 * ReverseIndexerCmd class that spins the indexer in the opposite direction in the
 * case of needing to eject a ball
 */
public class ReverseIndexerCmd extends CommandBase {
  private final Indexer indexer;

  /**
   * Creates a new ReverseIndxerCmd.
   * @param indexer Indexer that will be being used to rotate backwards
   */
  public ReverseIndexerCmd(Indexer indexer) {
    this.indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println(this.toString() + " started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Simply executes the {@code reverse()} function found in {@link Indexer}
   */
  @Override
  public void execute() {
    indexer.reverse();
  }

  // Called once the command ends or is interrupted.
  /**
   * Immediately stops the pivot once unscheduled or interrupted
   */
  @Override
  public void end(boolean interrupted) {
    indexer.stop();
    // System.out.println(this.toString() + "ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
