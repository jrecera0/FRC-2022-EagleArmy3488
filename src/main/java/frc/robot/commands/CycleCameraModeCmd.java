// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class CycleCameraModeCmd extends CommandBase {
  private final Limelight limelight;
  /** Creates a new CycleCameraModeCmd. */
  public CycleCameraModeCmd(Limelight limelight) {
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
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
  @Override
  public boolean isFinished() {
    return true;
  }
}
