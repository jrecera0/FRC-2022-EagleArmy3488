// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

/**
 * TargetBallCmd class that targets power cells on the field using the Limelight and
 * computer vision. See limelight docs for more info on field element targeting
 */
public class TargetBallCmd extends CommandBase {
  private final DriveTrain driveTrain;
  private final Limelight limelight;

  /**
   * Creates a new TargetBallCmd.
   * @param driveTrain Drivetrain to pivot in order to center the robot vision with the power cell
   * @param limelight Limelight to use for targeting and computer vision
   */
  public TargetBallCmd(DriveTrain driveTrain, Limelight limelight) {
    this.driveTrain = driveTrain;
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight); // exclude drivetrain to let arcade drive run
  }

  // Called when the command is initially scheduled.
  /**
   * Sets the limitlight to vision-viewing for processing; this
   * does obstruct driver view while targetting is occuring
   */
  @Override
  public void initialize() {
    limelight.setToVisionView();
    // System.out.println(this.toString() + " started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Using current values about {@code arcadeDrive()} in {@link DriveTrain}, the robot will
   * ultimately turn towards the targeted power cell while retaining normal driving functionality
   */
  @Override
  public void execute() {
    double rot = LimelightConstants.kP * limelight.getTargetXOffset();
    double fwd = driveTrain.getArcadeDriveFwdVal();
    if (limelight.isTargetToLeft()) {
      rot += LimelightConstants.kMinMovementSpeed;
    }
    else if (limelight.isTargetToRight()) {
      rot -= LimelightConstants.kMinMovementSpeed;
    }
    else {
      rot = 0;
    }
    driveTrain.arcadeDrive(fwd, rot);
  }

  // Called once the command ends or is interrupted.
  /**
   * Once command ends or is interrupted, the limelight is set back to the original driver
   * view so that the driver may use it as a normal camera
   */
  @Override
  public void end(boolean interrupted) {
    limelight.setToDriverView();
    // System.out.println(this.toString() + " ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
