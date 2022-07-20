// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

/**
 * CycleCameraModeCmd class that allows for the user to cycle between the different streaming
 * modes of the Limelight. I do not believe this was actually used within code.
 */
public class CycleCameraModeCmd extends CommandBase {
  private final Limelight limelight;

  /**
   * Creates a new CycleCameraModeCmd.
   * @param limelight Limelight that the Stream mode is being changed on
   */
  public CycleCameraModeCmd(Limelight limelight) {
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  /**
   * When scheduled, the init function will only run once to toggle between modes
   * 1 and 2 on the Limelight before the command immediately finishes, until scheduled
   * once more.
   */
  @Override
  public void initialize() {
    // System.out.println(this.toString() + " started!");
    if (limelight.getStreamNum() != 1) {
      limelight.setStreamingMode(1);
    } else {
      limelight.setStreamingMode(2);
    }
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
