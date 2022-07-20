// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

/**
 * SetPipelineCmd class that allows for the user to cycle between the different pipelines
 * of the Limelight.
 * 
 * Something tells me I intended to move the {@code limelight.setPipeline(pipelineNum.get())} method
 * call to {@code execute()} so it would be possible to dynamically update the pipeline, but I
 * have no means of testing this without a limelight so... if you ever reuse this command, consider
 * moving that line down and seeing if you can switch pipelines in ShuffleBoard on the fly.
 */
public class SetPipelineCmd extends CommandBase {
  private final Limelight limelight;
  private Supplier<Number> pipelineNum;

  /**
   * Creates a new SetPipelineCmd.
   * @param limelight Limelight to change the pipeline of
   * @param pipeline Number supplier to specify what pipeline number to be using
   */
  public SetPipelineCmd(Limelight limelight, Supplier<Number> pipeline) {
    this.limelight = limelight;
    pipelineNum = pipeline;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  /**
   * Sets the pipeline once when scheduled according to the number supplier
   */
  @Override
  public void initialize() {
    limelight.setPipeline(pipelineNum.get());
    // System.out.println(this.toString() + " started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * As of writing, nothing is in this method although I think what was written in {@code initalize()}
   * was supposed to go here.
   */
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
    return false;
  }
}
