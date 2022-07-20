// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.autoroutines.JsonTrajRoutine;
import frc.robot.autoroutines.CenterBallRoutine;
import frc.robot.autoroutines.FarBallRoutine;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.AutoShootCmd;
import frc.robot.commands.CycleCameraModeCmd;
import frc.robot.commands.DropPivotCmd;
import frc.robot.commands.IndexCmd;
import frc.robot.commands.PickupCmd;
import frc.robot.commands.RaisePivotCmd;
import frc.robot.commands.ReverseDTCmd;
import frc.robot.commands.ReverseIndexerCmd;
import frc.robot.commands.ReversePickupCmd;
import frc.robot.commands.RunClimberCmd;
import frc.robot.commands.RunLeftClimberCmd;
import frc.robot.commands.RunRightClimberCmd;
import frc.robot.commands.SetHighGoalCmd;
import frc.robot.commands.SetLowGoalCmd;
import frc.robot.commands.SetPipelineCmd;
import frc.robot.commands.SetToClimbSpeedCmd;
import frc.robot.commands.SetToExtendSpeedCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.SubsystemSpeedSetter;
import frc.robot.commands.TargetBallCmd;
import frc.robot.management.TrajectoryLoader;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Climber climber;
  private final DriveTrain driveTrain;
  private final Indexer indexer;
  private final Limelight limelight;
  private final Pickup pickup;
  private final Pivot pivot;
  private final Shooter shooter;

  private final XboxController xboxController;

  private final TrajectoryLoader trajectoryLoader;
  private final JsonTrajRoutine jsonTrajRoutine;
  private final CenterBallRoutine centerBallRoutine;
  private final FarBallRoutine farBallRoutine;

  /**
   * {@link SendableChooser} so we have the ability to pick an autonomous routine from our dashboard
   */
  private final SendableChooser<Command> autoRoutineChooser;
  // private final SendableChooser<Number> pipelineChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands. You essentiall assemble all the parts
   * of the robot in code here, from putting the subsystems all into one place, to binding all your buttons to running
   * specific commands.
  */
  public RobotContainer() {
    // Subsystems
    climber = new Climber();
    driveTrain = new DriveTrain();
    indexer = new Indexer();
    limelight = new Limelight();
    pickup = new Pickup();
    pivot = new Pivot();
    shooter = new Shooter();

    // Camera
    CameraServer.startAutomaticCapture();

    // Controller + bindings
    xboxController = new XboxController(ControllerConstants.kControllerPort);
    configureButtonBindings();

    // Trajectories and routines
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(DriveConstants.kVelocityMax, DriveConstants.kAccelerationMax);
    trajectoryConfig.setKinematics(driveTrain.getKinematics());
    trajectoryLoader = new TrajectoryLoader();
    jsonTrajRoutine = new JsonTrajRoutine(driveTrain, trajectoryLoader, shooter, indexer, pickup, pivot);
    centerBallRoutine = new CenterBallRoutine(trajectoryLoader, driveTrain, shooter, indexer, pickup, pivot);
    farBallRoutine = new FarBallRoutine(trajectoryLoader, driveTrain, shooter, indexer, pickup, pivot);

    // Dashboard choosers
    // Autonomous routine
    autoRoutineChooser = new SendableChooser<>();
    autoRoutineChooser.setDefaultOption("None", null);
    autoRoutineChooser.addOption("Json", jsonTrajRoutine.getCommand());
    autoRoutineChooser.addOption("CenterBall", centerBallRoutine.getCommand());
    autoRoutineChooser.addOption("FarBall", farBallRoutine.getCommand());

    // Actually put the data in smartdashboard!
    SmartDashboard.putData(autoRoutineChooser);

    // Default commands
    driveTrain.setDefaultCommand(new ArcadeDriveCmd(driveTrain, () -> -xboxController.getLeftY(), () -> xboxController.getRightX()));
  }

  /**
   * Use this method to define your button to command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Custom triggers
    Trigger leftTrigger = new Trigger(() -> xboxController.getLeftTriggerAxis() >= 0.1);
    Trigger rightTrigger = new Trigger(() -> xboxController.getRightTriggerAxis() >= 0.1);

    // Button bindings please clean this up james there's too much typing
    // Primary Controls
    leftTrigger.whileActiveOnce(new PickupCmd(pickup));
    new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value).and(
      new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
      ).whileActiveOnce(new IndexCmd(indexer));
    new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
      .whileActiveOnce(new AutoShootCmd(indexer, shooter, pickup));
    new JoystickButton(xboxController, XboxController.Button.kB.value)
      .whileActiveOnce(new TargetBallCmd(driveTrain, limelight));
    new JoystickButton(xboxController, XboxController.Button.kY.value)
      .whileActiveOnce(new SetPipelineCmd(limelight, () -> LimelightConstants.kRedPipeline));
    new JoystickButton(xboxController, XboxController.Button.kA.value)
      .whileActiveOnce(new SetPipelineCmd(limelight, () -> LimelightConstants.kBluePipeline));
    new JoystickButton(xboxController, XboxController.Button.kX.value)
      .whenPressed(new DropPivotCmd(pivot).withTimeout(1));
    new JoystickButton(xboxController, XboxController.Button.kLeftStick.value)
      .whileActiveOnce(new ReverseDTCmd(driveTrain));
    new JoystickButton(xboxController, XboxController.Button.kRightStick.value)
      .whileActiveOnce(new CycleCameraModeCmd(limelight));
    new JoystickButton(xboxController, XboxController.Button.kStart.value) // RIGHT ONE
      .whileActiveOnce(new DropPivotCmd(pivot));
    new JoystickButton(xboxController, XboxController.Button.kBack.value) // LEFT ONE
      .whileActiveOnce(new SetLowGoalCmd(shooter));

    // Independent climber control
    new JoystickButton(xboxController, XboxController.Button.kStart.value)
      .and(new JoystickButton(xboxController, XboxController.Button.kY.value))
      .whileActiveOnce(new SetToExtendSpeedCmd(climber).andThen(new RunLeftClimberCmd(climber)));
    new JoystickButton(xboxController, XboxController.Button.kStart.value)
      .and(new JoystickButton(xboxController, XboxController.Button.kA.value))
      .whileActiveOnce(new SetToClimbSpeedCmd(climber).andThen(new RunLeftClimberCmd(climber)));
    new JoystickButton(xboxController, XboxController.Button.kBack.value)
      .and(new JoystickButton(xboxController, XboxController.Button.kY.value))
      .whileActiveOnce(new SetToExtendSpeedCmd(climber).andThen(new RunRightClimberCmd(climber)));
    new JoystickButton(xboxController, XboxController.Button.kBack.value)
      .and(new JoystickButton(xboxController, XboxController.Button.kY.value))
      .whileActiveOnce(new SetToClimbSpeedCmd(climber).andThen(new RunRightClimberCmd(climber)));

    // Secondary Controls
    rightTrigger.and(leftTrigger).whileActiveOnce(new ReversePickupCmd(pickup));
    rightTrigger.and(new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value))
      .whileActiveOnce(new ReverseIndexerCmd(indexer));
    rightTrigger.and(new JoystickButton(xboxController, XboxController.Button.kRightBumper.value))
      .whileActiveOnce(new ShootCmd(shooter));
    rightTrigger.and(new JoystickButton(xboxController, XboxController.Button.kY.value))
      .whileActiveOnce(new SetToExtendSpeedCmd(climber).andThen(new RunClimberCmd(climber)));
    rightTrigger.and(new JoystickButton(xboxController, XboxController.Button.kA.value))
      .whileActiveOnce(new SetToClimbSpeedCmd(climber).andThen(new RunClimberCmd(climber)));
    rightTrigger.and(new JoystickButton(xboxController, XboxController.Button.kX.value))
      .whenActive(new RaisePivotCmd(pivot).withTimeout(1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    driveTrain.resetOdometry(new Pose2d());
    return autoRoutineChooser.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getTeleopCommand() {
    return new SubsystemSpeedSetter(indexer, pickup, shooter, climber, xboxController);
  }
}
