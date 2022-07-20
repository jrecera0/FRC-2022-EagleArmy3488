// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/**
 * ResetDTPoseCmd class that resets the odometry of the {@link DriveTrain} to aid
 * in trajectory following
 */
public class ResetDTPoseCmd extends CommandBase {
  DriveTrain driveTrain;

  /**
   * Creates a new ReverseDTCmd.
   * @param driveTrain Drivetrain to reset the odometry of
   */
  public ResetDTPoseCmd(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  /**
   * When scheduled, the odometry of the passed in {@link DriveTrain} will be reset to
   * a new {@link Pose2d}.
   */
  @Override
  public void initialize() {
    // driveTrain.invertDriveTrain();
    driveTrain.resetOdometry(new Pose2d());
    // System.out.println(this.toString() + " started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println(this.toString() + " ended!");
  }

  // Returns true when the command should end.
  /**
   * This is a single action command so it should immediately finish once scheduled, as
   * we only need to reset odometry once in {@code initialize()}
   */
  @Override
  public boolean isFinished() {
    return true;
  }
}
