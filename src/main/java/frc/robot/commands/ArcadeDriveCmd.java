// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/**
 * ArcadeDriveCmd class that allows for the robot to drive in arcade drive
 */
public class ArcadeDriveCmd extends CommandBase {
  private final DriveTrain driveTrain;
  private final Supplier<Double> speedFunc, rotationFunc; // I'm new to suppliers

  /**
   * Creates a new ArcadeDriveCmd.
   * @param driveTrain Drivetrain that is being driven
   * @param speedFunc Function that returns a double in order to specify the fwd-speed of the robot
   * @param rotationFunc Function that returns a double in order to specify the rot speed of the robot
   */
  public ArcadeDriveCmd(DriveTrain driveTrain, Supplier<Double> speedFunc, Supplier<Double> rotationFunc) {
    this.speedFunc = speedFunc;
    this.rotationFunc = rotationFunc;
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.s
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println(this.toString() + " started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Using the values from the double suppliers, we store these into a variable and pass them into the
   * {@code arcadeDrive()} method of {@link DriveTrain}.
   */
  @Override
  public void execute() {
    double fwd = speedFunc.get();
    double rot = rotationFunc.get();
    driveTrain.arcadeDrive(fwd, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println(this.toString() + " ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
