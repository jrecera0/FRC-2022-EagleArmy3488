// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Shooter;

/**
 * AutoShootCmd class that allows for us to automatically index and shoot the ball
 * when the shooting trigger is pressed. This removes the need to manually index the ball
 * towards the shooter while waiting for it to spin up to speed.
 */
public class AutoShootCmd extends CommandBase {
  private final Indexer indexer;
  private final Shooter shooter;
  private final Pickup pickup;
  
  /**
   * Creates a new AutoShootCmd.
   * @param indexer Indexer that will roll the power cells to the shooter
   * @param shooter Shooter that will shoot the power cells towards the hub
   * @param pickup Pickup that will also rotate in case any power cells are stuck towards the front
   */
  public AutoShootCmd(Indexer indexer, Shooter shooter, Pickup pickup) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.pickup = pickup;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer, shooter, pickup);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println(this.toString() + " started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * When scheduled, the shooter will always be spinning, and while this is happening,
   * the robot will check if we have reached the threshold speed to start moving power cells
   * towards the shooter to shoot them out. If so, we run the indexer and the pickup, otherwise
   * they will be in a stopped state until the "ready" threshold state is reached.
   */
  @Override
  public void execute() {
    shooter.shoot();
    if (shooter.getVelocity() >= shooter.getThreshValue()) {
      indexer.index();
      pickup.pickup();
      // System.out.println("Indexing...");
    }
    else {
      indexer.stop();
      pickup.stop();
      // System.out.println("Not indexering...");
    }
  }

  // Called once the command ends or is interrupted.
  /**
   * Immediately stop all subsystems once unscheduled or interrupted
   */
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    indexer.stop();
    pickup.stop();
    // System.out.println(this.toString() + " ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
