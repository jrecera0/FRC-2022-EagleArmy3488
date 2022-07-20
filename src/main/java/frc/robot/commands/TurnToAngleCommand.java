// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * TurnToAngleCommand class that turns the robot to a given heading for the autonomous period.
 * This ended up not being used and was a class found online, it definitely has some use for
 * the future but within this specific project, has no applied usage as we forgoed making an
 * autonomous routine with rotations and pivoting.
 */
public class TurnToAngleCommand extends CommandBase {
  /** Creates a new TurnToAngleCommand. */
  
  private final DriveTrain m_robotDrive;
  private boolean complete = false;
  private double angle;
  private Timer timer = new Timer();
  private double timeout;
  public TurnToAngleCommand(DriveTrain subsystem, double degrees, double timeoutS){
      m_robotDrive = subsystem;
      angle = degrees;
      timeout = timeoutS;
      addRequirements(subsystem);
  }

  @Override
  public void initialize(){
      timer.reset();
      timer.start();
      complete = false;
  }
  
  @Override
  public void execute(){
      double gyroAngle = m_robotDrive.getHeading().getDegrees();

      final double kP = 0.005;
      SmartDashboard.putNumber("gyroAngle", gyroAngle);
  
      if (angle > 180) {
          angle = -(360 - angle);
      } else if (angle < -180) {
          angle = 360 + angle;
      }
  
      double err = angle - gyroAngle;
  
      double speed = MathUtil.clamp(err * kP, -0.4, 0.4);
  
      if (Math.abs(err) > 2 && timer.get() < timeout) {
          m_robotDrive.arcadeDrive(0, -speed);
      } else {
          complete = true;
      }
  }

  @Override
  public void end(boolean inturrupted){
      m_robotDrive.arcadeDrive(0, 0);
      timer.stop();
  }

  @Override
  public boolean isFinished(){
      return complete;
  }
}