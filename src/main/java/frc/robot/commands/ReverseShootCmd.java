// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

/**
 * ReverseShootCmd class that spins the shooter in the opposite direction in the case of needing
 * to eject a ball, although it is unlikely that this will actually help unlike {@link ReverseIndexerCmd}
 * or {@link ReversePickupCmd}.
 */
public class ReverseShootCmd extends CommandBase {
  private final Shooter shooter;

  /**
   * Creates a new ReverseShootCmd
   * @param shooter Shooter that will be being used to rotate backwards
   */
  public ReverseShootCmd(Shooter shooter) {
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  /**
   * Sets the shooter to the negative value of its specified speed, as a reverse function was
   * not implemented within its class
   */
  @Override
  public void initialize() {
    shooter.setSpeed(-shooter.getSpeedValue());
    // System.out.println(this.toString() + " started!");
  }

  // Called every time the scheduler runs while the command is scheduled.

  /**
   * Sets the shooter to "shoot", although now in the opposite direction because of the manual
   * speed set.
   */
  @Override
  public void execute() {
    shooter.shoot();
  }

  // Called once the command ends or is interrupted.
  /**
   * Resets the shooter speed back to its default value before immediately stopping when
   * unscheduled or interrupted
   */
  @Override
  public void end(boolean interrupted) {
    // reset speed back to what it was originally
    shooter.setSpeed(-shooter.getSpeedValue());
    shooter.stop();
    // System.out.println(this.toString() + " ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
