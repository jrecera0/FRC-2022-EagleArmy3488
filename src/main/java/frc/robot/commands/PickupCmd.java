// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pickup;

/**
 * PickupCmd class that runs the pickup at normal speed to pick up power cells from the field
 */
public class PickupCmd extends CommandBase {
  private final Pickup pickup;

  /**
   * Creates a new PickupCmd.
   * @param pickup Pickup that is being used to rotate and pick up balls
   */
  public PickupCmd(Pickup pickup) {
    this.pickup = pickup;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pickup);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println(this.toString() + " started!");
  }

  // Called every time the scheduler runs while the command is scheduled
  /**
   * When scheduled, run the pickup mechanism via the {@code pickup()} method
   */
  @Override
  public void execute() {
    pickup.pickup();
  }

  // Called once the command ends or is interrupted.

  /**
   * Immediately stops the pickup once unscheduled or interrupted.
   */
  @Override
  public void end(boolean interrupted) {
    pickup.stop();
    // System.out.println(this.toString() + " ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
