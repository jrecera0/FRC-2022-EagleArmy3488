// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PickupConstants;
import frc.robot.subsystems.Pickup;

public class FastPickupCmd extends CommandBase {
  private final Pickup pickup;

  /** Creates a new PickupCmd. */
  public FastPickupCmd(Pickup pickup) {
    this.pickup = pickup;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pickup);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pickup.setSpeed(1.0);
    // System.out.println(this.toString() + " started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pickup.pickup();
  }

  // Called once the command ends or is interrupted.
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
