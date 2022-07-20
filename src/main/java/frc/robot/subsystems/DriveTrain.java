// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * DriveTrain class that consists of four motors assembled in a tank drive configuration using the
 * white 6" (dia.) wheels, with all set up done in order to have working odometry for accurate
 * trajectory following.
 * 
 * <p> There is a lot that goes into getting the robot odometry all set up, and hopefully you won't
 * have to rewrite this class too much in the future. For deeper explanation and detailing, the FRC
 * docs get into setting up odometry in relatively clear detail, although with different motors and
 * encoder configurations. In principle, it is all relatively the same. The main idea here
 * is that we are able to get all information about our robot's heading, position, and veloicty at
 * any given moment, and then use this to follow a path or trajectory during the autonomous period.
 * Without robot odometry, a lot of the things we do in autonomous becomes trickier or impossible so
 * it's important that this gets set up correctly.
 * 
 * <p> Note on the gyro: There's a commented import because we were alternating between the USB
 * and RoboRIO port for the gyro we were using, just make sure to use the right one depending on
 * whichever one you end up using. This was something I read but be careful if you're using anything
 * else on USB besides the gyro because it will potentially increase latency or make it take longer
 * to get any info on the robot heading.
 * 
 * <p> Other notes: There is a somewhat janky function that inverts the drive train, it works but
 * something about it definitely feels hacky. Be careful when using it? Hasn't caused issues for me
 * but just in case something wasn't reversed and odometry is wrong as a result of using it, make
 * sure to keep an eye on everything when using it. Also, note that there's a little bit of math and
 * conversion from raw encoder units to m/s. I don't believe in the future there's a reason why they
 * would change the unit that trajectory following in handled in but just note that if something changes,
 * e.g. wheel radius, encoder resolution, trajectory parsing, gear ratio, the math could vary just a
 * bit. The good thing is that a lot of this can be accounted for in {@link frc.robot.Constants}, the
 * bad news is that I can't account for every last little thing, so... best of luck, I pray nothing
 * drastic changes that messes up this class and requires a complete rewrite ;^;
 * 
 * <p> Also I think I stored the {@code fwd} and {@code rot} values of the robot so we could use the
 * limelight for targetting while driving? Also also note that {@code SpeedControllerGroup} is no longer
 * being used in favor of using the master-slave configuration of motors for control and grouping.
 */
public class DriveTrain extends SubsystemBase {
  private AHRS gyro;
  private DifferentialDrive driveTrain;
  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry odometry;
  private PIDController leftPIDController, rightPIDController;
  private SimpleMotorFeedforward feedForward;
  private WPI_TalonFX frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
  private boolean isLeftSideInverted;
  private boolean isRightSideInverted;
  private boolean gyroReversed;
  private boolean isReversed;

  private double fwd;
  private double rot;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    frontLeftMotor = new WPI_TalonFX(kFrontLeftMotorID);
    frontRightMotor = new WPI_TalonFX(kFrontRightMotorID);
    backLeftMotor = new WPI_TalonFX(kBackLeftMotorID);
    backRightMotor =  new WPI_TalonFX(kBackRightMotorID);
    setFactory(frontLeftMotor);
    setFactory(frontRightMotor);
    setFactory(backLeftMotor);
    setFactory(backRightMotor);
    frontLeftMotor.setInverted(kIsLeftSideInverted);
    backLeftMotor.setInverted(kIsLeftSideInverted);
    frontRightMotor.setInverted(kIsRightSideInverted);
    backRightMotor.setInverted(kIsRightSideInverted);
    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);
    frontLeftMotor.configOpenloopRamp(kRampInSec);
    frontRightMotor.configOpenloopRamp(kRampInSec);
    backLeftMotor.configOpenloopRamp(kRampInSec);
    backRightMotor.configOpenloopRamp(kRampInSec);
    setDriveMode(NeutralMode.Brake);

    driveTrain = new DifferentialDrive(frontLeftMotor, frontRightMotor);
    driveTrain.setSafetyEnabled(false);

    gyro = new AHRS(SPI.Port.kMXP); // SerialPort.Port.kUSB for USB, SPI.Port.kMXP for rio
    
    resetEncoders(); // make sure odometry isn't screwed up

    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(kTrackWidth));
    odometry = new DifferentialDriveOdometry(getHeading());
    feedForward = new SimpleMotorFeedforward(kS, kV, kA);
    leftPIDController = new PIDController(kP, kI, kD);
    rightPIDController = new PIDController(kP, kI, kD);

    isLeftSideInverted = kIsLeftSideInverted;
    isRightSideInverted = kIsRightSideInverted;
    gyroReversed = kGyroReversed;
    isReversed = false;
  }

  /**
   * Pushes current velocity, heading, and voltage of the robot to ShuffleBoard
   */
  @Override
  public void periodic() {
    odometry.update(getHeading(), getLeftWheelDistance(), getRightWheelDistance());
    SmartDashboard.putNumber("LeftDTVelocity", getDriveWheelSpeeds().leftMetersPerSecond);
    SmartDashboard.putNumber("RightDTVelocity", getDriveWheelSpeeds().rightMetersPerSecond);
    SmartDashboard.putNumber("RobotHeading", getHeading().getDegrees());
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
  }

  // Feeds for autonomous
  // Post-Comp James: I have no idea what the heck the comment above this one is supposed to be

  // Controlling DriveTrain
  /**
   * Drive the robot in an arcade drive control scheme where one input value controls
   * the forwards-backwards motion of the robot, and the other controls the rotational
   * speed of the robot.
   * @param fwd speed of the robot moving forwards or backwards
   * @param rot rotational speed of the robot
   */
  public void arcadeDrive(double fwd, double rot) {
    this.fwd = fwd;
    this.rot = rot;
    driveTrain.arcadeDrive(this.fwd, this.rot);
  }

  /**
   * Gets the current stored fwd speed of the robot.
   * @return Current fwd speed of {@code arcadeDrive()}
   */
  public double getArcadeDriveFwdVal() {
    return fwd;
  }

  /**
   * Gets the current stored rot speed of the robot.
   * @return Current rot speed of {@code arcadeDrive()}
   */
  public double getArcadeDriveRotVal() {
    return rot;
  }

  /**
   * Drive the robot in a tank drive control scheme where each input controls
   * the speed of a specific side of the robot
   * @param left speed of the left side of the robot
   * @param right speed of the right side of the robot
   */
  public void tankDrive(double left, double right) {
    driveTrain.tankDrive(left, right);
  }

  /**
   * Sets the voltage of each side of the robot to more accurately drive around. This has mainly
   * been used for trajectory following and I have yet to use this elsewhere.
   * @param leftVolts voltage to apply to the left side of the robot
   * @param rightVolts voltage to apply to the right side of the robot
   */
  public void setTankDriveVolts(double leftVolts, double rightVolts) {
    frontLeftMotor.setVoltage(leftVolts);
    frontRightMotor.setVoltage(rightVolts);
    driveTrain.feed();
  }

  /**
   * Sets a maximum voltage that can be applied to the motors. Presumably limits the
   * maximum speed they can be driven at.
   * @param maxOutput Maximum voltage that can be applied to the motors
   */
  public void setMaxOutput(double maxOutput) {
    driveTrain.setMaxOutput(maxOutput);
  }

  /**
   * Set the drive mode of the robot's drive train
   * @param driveMode Drive mode to apply (Brake or Coast)
   */
  public void setDriveMode(NeutralMode driveMode) {
    frontLeftMotor.setNeutralMode(driveMode);
    frontRightMotor.setNeutralMode(driveMode);
    backLeftMotor.setNeutralMode(driveMode);
    backRightMotor.setNeutralMode(driveMode);
  }

  /**
   * Resets the passed motor to factory values to avoid lingering values from interfering with
   * the config we need
   * @param motor Motor to reset to factor values
   */
  public void setFactory(WPI_TalonFX motor) {
    motor.configFactoryDefault();
  }

  // Obtaining DriveTrain Information
  /**
   * Get the current speeds of the wheels of the drive train, converting from raw encoder units
   * to m/s to properly report for trajectory following
   * @return Speeds of the drive train wheels in m/s
   */
  public DifferentialDriveWheelSpeeds getDriveWheelSpeeds() {
    // returns wheel speeds in m/s (meters per second)
    double leftRotationsPerSecond = (double) getLeftEncoderVelocity() / kEncoderResolution / kGearRatio * 10;        
    double leftVelocity = leftRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(kWheelRadius);
    double rightRotationsPerSecond = (double) getRightEncoderVelocity() / kEncoderResolution / kGearRatio * 10;        
    double rightVelocity = rightRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(kWheelRadius);
    return new DifferentialDriveWheelSpeeds(leftVelocity, rightVelocity);
  }

  /**
   * Gets the current heading of the robot in Radians for trajectory following
   * @return Heading of the robot
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees((gyroReversed) ? -gyro.getAngle() : gyro.getAngle());
  }

  /**
   * Gets the total distance in meters that the left side of the robot has driven since it has been last reset
   * @return Total distance left side has driven
   */
  public double getLeftWheelDistance() {
    double leftDistance = ((double) getLeftEncoderPosition()) / kEncoderResolution / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadius);
    return leftDistance;
  }

  /**
   * Gets the total distance in meters that the right side of the robot has driven since it has been last reset
   * @return Total distance right side has driven
   */
  public double getRightWheelDistance() {
    double rightDistance = ((double) getRightEncoderPosition()) / kEncoderResolution / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadius);
    return rightDistance;
  }

  /**
   * Gets the raw encoder positioning of the left side of the robot since it has been last reset
   * @return Raw encoder position of the robot's left side
   */
  public double getLeftEncoderPosition() {
    return frontLeftMotor.getSelectedSensorPosition();
  }

  /**
   * Gets the raw encoder positioning of the right side of the robot since i thas been last reset
   * @return Raw encoder position of the robot's right side
   */
  public double getRightEncoderPosition() {
    return frontRightMotor.getSelectedSensorPosition();
  }

  /**
   * Gets the raw encoder velocity of the left side of the robot since it has been last reset
   * @return Raw encoder velocity of the robot's left side
   */
  public double getLeftEncoderVelocity() {
    return frontLeftMotor.getSelectedSensorVelocity();
  }

  /**
   * Gets the raw encoder velocity of the right side of the robot since it has been last reset
   * @return Raw encoder velocity of the robot's right side
   */
  public double getRightEncoderVelocity() {
    return frontRightMotor.getSelectedSensorVelocity();
  }

  /**
   * Gets the kinematics of the robot's drive train
   * @return Robot's kinematics
   */
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Gets the odometry of the robot's drive train
   * @return Robot's odometry
   */
  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  /**
   * Gets the feedfoward loop of the robot in order to allow for more accurate robot movement
   * in trajectory following
   * @return Robot's feedforward
   */
  public SimpleMotorFeedforward getSimpleMotorFeedForward() {
    return feedForward;
  }

  /**
   * Gets the PID Controller of the left side of the robot for trajectory following
   * @return PID Controller of robot's left side
   */
  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  /**
   * Gets the PID Controller of the right side of the robot for trajectory following
   * @return PID Controller of robot's right side
   */
  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  /**
   * Gets the current position of the robot
   * @return Robot's pose
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Gets the current heading of the robot in degrees
   * @return The current robot heading
   */
  public double getGyroAngle() {
    return gyroReversed ? -gyro.getAngle() : gyro.getAngle();
  }

  // Reset Functions
  /**
   * Resets the odometry of the robot and allows for a custom starting pose to be set
   * @param pose Position to start the robot in after resetting
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    resetGyro();
    System.out.println("WARNING! " + getHeading());
    odometry.resetPosition(pose, getHeading());
  }

  /**
   * Resets the motor encoders to 0
   */
  public void resetEncoders() {
    frontLeftMotor.setSelectedSensorPosition(0);
    frontRightMotor.setSelectedSensorPosition(0);
  }

  /**
   * Resets the gyro heading to 0
   */
  public void resetGyro() {
    gyro.reset();
  }

  /**
   * Reverses the entire drive train so the robot has the ability to drive backwards
   */
  public void invertDriveTrain() {
    isLeftSideInverted = !isLeftSideInverted;
    isRightSideInverted = !isRightSideInverted;
    gyroReversed = !gyroReversed;
    frontLeftMotor.setInverted(isLeftSideInverted);
    backLeftMotor.setInverted(isLeftSideInverted);
    frontRightMotor.setInverted(isRightSideInverted);
    backRightMotor.setInverted(isRightSideInverted);
    isReversed = !isReversed;
    if (isReversed) {
      driveTrain = new DifferentialDrive(frontRightMotor, frontLeftMotor);
      driveTrain.setSafetyEnabled(false);
    } else {
      driveTrain = new DifferentialDrive(frontLeftMotor, frontRightMotor);
      driveTrain.setSafetyEnabled(false);
    }
  }
}
