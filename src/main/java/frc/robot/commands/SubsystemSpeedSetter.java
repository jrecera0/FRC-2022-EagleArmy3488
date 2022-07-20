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

/**
 * SubsystemSpeedSetting class that allows for us to manually set the speeds of each
 * individual subsystem on the fly. The writing of this was definitely a little bit more
 * involved and definitely needs some improvements, but the functionality of it is as follows...
 * 
 * <p> Manually passing in all subsystems (should probably find a way to just pass these in automatically
 * rather than having to add a new subsystem every time a new mechanism gets added in code) and
 * the Xbox controller (this is definitely the incorrect way of doing things according to the command-based
 * paradigm, we should actually be binding the button presses by hand in {@link frc.robot.RobotContainer}
 * rather than polling them in here), we track the current and previous button presses of the D-Pad
 * from the controller, and using that we are able to increase or decrease the current speed (Up or Down)
 * and cycle through all the subsystems according to a hardcoded enum with a list of all the subsystems,
 * and hard-linking them to a certain index of a list (Left and Right cycles these subsystems).
 * The biggest issue is that this is really a custom class in that everything is pretty much hard-coded
 * and likely can't be reused for the future unless for whatever reason the subsystems and functionality
 * remains largely the same, so I do recommend that a rewrite of this be done so that it isn't
 * subsystem specific and can be applied to whatever robot is being used in the future, because it
 * is VERY handy having a class like this availble to tweak on the fly, its just that this specific
 * one was not designed with future robustness in mind. The principles will remain, but the code
 * itself is likely to break with changing requirements over time.
 */
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

  /**
   * Creates a new CustomSpeedMgmtCmd.
   * @param indexer Indexer to manage the speed of
   * @param pickup Pickup to manage the speed of
   * @param shooter Shooter to manage the speed of (there's custom speed management for this as the -1.0 to 1.0 is not in use)
   * @param climber Climber to manage the speed of
   * @param xbox XboxController to use to poll for button presses
   */
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
  /**
   * I will now attempt to explain what half of this code does, it's mainly just a lot of if-statements
   * and then cycling through the different subsystems.
   * 
   * <p> First, we poll for button presses from the D-Pad, seeing if we've pressed "left" or "right". Depending
   * on the button press and where we are according to the list of subsystems we are cycling through, we either increment,
   * decrement, or jump to a certain index of the list so we can establish subsystem we are adjusting the speed of.
   * 
   * <p> We apply the speed then to the specifc subsystem, making sure to grab the current speed of each subsystem
   * first before changing them from here. Speed adjustments are done via the "up" or "down" buttons of the D-Pad,
   * and have different behavior depending on if you are adjusting normal subsystems or the shooter, as the shooter
   * has a very different scaling in speed versus the default -1.0 to 1.0 power set up for all other subsystems.
   * There is also a custom toggle to go straight to the climber by pressing on the right joystick, but this was never
   * used in competition as we were already able to climb very quickly with our configuration.
   * 
   * <p> As mentioned before, there is definitely a better way to write all this, so I do recommend a rewrite
   * of this class for the future for robustness and for lessened complexity than this mess of if-statements
   * to manage each individual subsystem.
   */
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
