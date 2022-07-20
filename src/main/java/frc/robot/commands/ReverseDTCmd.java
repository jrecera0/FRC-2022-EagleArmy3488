// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/**
 * ReverseDTCmd class that reverses the entire drivetrain of the robot to now drive backwards
 * both in autonomous and in normal driving as well, so that the back becomes the front and the
 * front becomes the back
 */
public class ReverseDTCmd extends CommandBase {
  DriveTrain driveTrain;
  /**
   * Creates a new ReverseDTCmd.
   * @param driveTrain Drivetrain that will be reversed
   */
  public ReverseDTCmd(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  /**
   * When scheduled, the init function will run once to invert the drivetrain according to
   * {@code invertDriveTrain()}. Can be scheduled multiple times to invert the drivetrain once
   * more.
   */
  @Override
  public void initialize() {
    driveTrain.invertDriveTrain();
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
   * This is set to true so as to immediately finish the command once scheduled, as the task
   * only needs to be run once in {@code initialize()}
   */
  @Override
  public boolean isFinished() {
    return true;
  }
}
