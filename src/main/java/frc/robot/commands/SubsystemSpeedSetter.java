// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Shooter;

public class SubsystemSpeedSetter extends CommandBase {
  // State management for the speed setting
  private enum State {
    INTAKE,
    INDEXER,
    SHOOTER,
    CLIMBER
  }

  // low-key redudant?
  private State[] state = {
    State.INTAKE,
    State.INDEXER,
    State.SHOOTER,
    State.CLIMBER
  };

  // Declare subsystems here
  private final Pickup pickup;
  private final Indexer indexer;
  private final Shooter shooter;
  private final Climber climber;

  // Xbox controller
  private final XboxController xbox;

  // Management variables
  private int prevPOV = -1;
  private int prevIdx = 2;
  private int stateIdx = 2;
  private double speed = 0;

  /** Creates a new CustomSpeedMgmtCmd. */
  public SubsystemSpeedSetter(Indexer indexer, Pickup pickup, Shooter shooter, Climber climber, XboxController xbox) {
    // Setting up subsystems
    this.indexer = indexer;
    this.shooter = shooter;
    this.pickup = pickup;
    this.climber = climber;
    this.xbox = xbox;

    // Setting init as pickup speed as that this the first subsystem
    speed = shooter.getSpeedValue();

    // addRequirements() is NOT used so as to keep subsystems free for other commands
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.toString() + " started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Polling button press for change in state
    if (xbox.getPOV() == 270 && prevPOV != xbox.getPOV()) {
      stateIdx--;
      if (stateIdx < 0) {
        stateIdx = state.length - 1;
      }
      prevPOV = xbox.getPOV();
      System.out.println("Speed management subsystem: " + state[stateIdx].toString());
    }

    if (xbox.getPOV() == 90 && prevPOV != xbox.getPOV()) {
      stateIdx++;
      if (stateIdx > state.length - 1) {
        stateIdx = 0;
      }
      prevPOV = xbox.getPOV();
      System.out.println("Speed management subsystem: " + state[stateIdx].toString());
    }

    // Changing test speed
    if (xbox.getPOV() == 0 && prevPOV != xbox.getPOV()) {
      if (stateIdx != 2) {
        // Normal speed setting
        speed = TestConstants.kSpeedIncrement + speed >= 1.0 ? 1.0 : TestConstants.kSpeedIncrement + speed;
      }
      else {
        // Custom speed setting for the shooter
        speed += TestConstants.kCustomSpeedIncrement;
      }
      prevPOV = xbox.getPOV();
      System.out.println("Speed set to: " + speed);
    }

    if (xbox.getPOV() == 180 && prevPOV != xbox.getPOV()) {
      if (stateIdx != 2) {
        // Normal speed setting
        speed = speed - TestConstants.kSpeedIncrement <= -1.0 ? -1.0 : speed - TestConstants.kSpeedIncrement;
      }
      else {
        // Custom speed setting again
        speed -= TestConstants.kCustomSpeedIncrement;
      }
      prevPOV = xbox.getPOV();
      System.out.println("Speed set to: " + speed);
    }

    // Reset button press polling
    if (xbox.getPOV() == -1 && prevPOV != xbox.getPOV()) {
      prevPOV = xbox.getPOV();
    }

    // Custom toggle for if we want to quickly switch to climber
    if (xbox.getRightStickButtonPressed()) {
      stateIdx = 3; // indexer of the climber
    }
    
    // Speed management for Intake
    if (state[stateIdx] == State.INTAKE) {
      if (prevIdx != stateIdx) {
        speed = pickup.getSpeedValue();
        prevIdx = stateIdx;
        System.out.println("Intake speed: " + speed);
      }
      pickup.setSpeed(speed);
    }

    // Speed maangement for Indexer
    if (state[stateIdx] == State.INDEXER) {
      if (prevIdx != stateIdx) {
        speed = indexer.getSpeedValue();
        prevIdx = stateIdx;
        System.out.println("Indexer speed: " + speed);
      }
      indexer.setSpeed(speed);
    }

    // Speed management for both shooter motors
    if (state[stateIdx] == State.SHOOTER) {
      if (prevIdx != stateIdx) {
        speed = shooter.getSpeedValue();
        prevIdx = stateIdx;
        System.out.println("Shooter speed: " + speed);
      }
      shooter.setSpeed(speed);
    }

    // Speed management for climber (both motors)
    if (state[stateIdx] == State.CLIMBER) {
      if (prevIdx != stateIdx) {
        speed = climber.getSpeedValue();
        prevIdx = stateIdx;
        System.out.println("Climber speed: " + speed);
      }
      climber.setSpeed(speed);
    }

    // Because we need info on shuffleboard to visibly see
    SmartDashboard.putString("SpeedSetterSubsystem", state[stateIdx].toString());
    SmartDashboard.putNumber("SpeedSetterSubsystemSpeed", speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.toString() + " ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
