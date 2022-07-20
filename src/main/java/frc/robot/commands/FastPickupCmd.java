// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PickupConstants;
import frc.robot.subsystems.Pickup;

/**
 * FastPickupCmd class that runs the pickup at full speed to pick up
 * power cells from the field.
 */
public class FastPickupCmd extends CommandBase {
  private final Pickup pickup;

  /**
   * Creates a new FastPickupCmd.
   * @param pickup Pickup that is being used to rotate and pick up balls
   */
  public FastPickupCmd(Pickup pickup) {
    this.pickup = pickup;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pickup);
  }

  // Called when the command is initially scheduled.

  /**
   * Initially overrides the speed of the pickup mechanism to 1.0 instead of what is set in
   * the {@link frc.robot.Constants} class.
   */
  @Override
  public void initialize() {
    pickup.setSpeed(1.0);
    // System.out.println(this.toString() + " started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * While scheduled, run the pickup mechanism via the {@code pickup()} method
   */
  @Override
  public void execute() {
    pickup.pickup();
  }

  // Called once the command ends or is interrupted.
  /**
   * Resets the speed to whatever value it is set to within the {@link frc.robot.Constants} class
   * before then immediately stopping the pickup.
   */
  @Override
  public void end(boolean interrupted) {
    pickup.setSpeed(PickupConstants.kPickupSpeed);
    pickup.stop();
    // System.out.println(this.toString() + " ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
