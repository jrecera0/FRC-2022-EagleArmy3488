// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Shooter;

public class AutoShootCmd extends CommandBase {
  private final Indexer indexer;
  private final Shooter shooter;
  private final Pickup pickup;
  
  /** Creates a new AutoShoot. */
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
