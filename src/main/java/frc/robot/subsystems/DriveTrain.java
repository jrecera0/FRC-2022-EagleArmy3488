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

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
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

  @Override
  public void periodic() {
    odometry.update(getHeading(), getLeftWheelDistance(), getRightWheelDistance());
    SmartDashboard.putNumber("LeftDTVelocity", getDriveWheelSpeeds().leftMetersPerSecond);
    SmartDashboard.putNumber("RightDTVelocity", getDriveWheelSpeeds().rightMetersPerSecond);
    SmartDashboard.putNumber("RobotHeading", getHeading().getDegrees());
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
  }

  // Feeds for autonomous
  // Controlling DriveTrain  
  public void arcadeDrive(double fwd, double rot) {
    this.fwd = fwd;
    this.rot = rot;
    driveTrain.arcadeDrive(this.fwd, this.rot);
  }

  public double getArcadeDriveFwdVal() {
    return fwd;
  }

  public double getArcadeDriveRotVal() {
    return rot;
  }

  public void tankDrive(double left, double right) {
    driveTrain.tankDrive(left, right);
  }

  public void setTankDriveVolts(double leftVolts, double rightVolts) {
    frontLeftMotor.setVoltage(leftVolts);
    frontRightMotor.setVoltage(rightVolts);
    driveTrain.feed();
  }

  public void setMaxOutput(double maxOutput) {
    driveTrain.setMaxOutput(maxOutput);
  }

  public void setDriveMode(NeutralMode driveMode) {
    frontLeftMotor.setNeutralMode(driveMode);
    frontRightMotor.setNeutralMode(driveMode);
    backLeftMotor.setNeutralMode(driveMode);
    backRightMotor.setNeutralMode(driveMode);
  }

  public void setFactory(WPI_TalonFX motor) {
    motor.configFactoryDefault();
  }

  // Obtaining DriveTrain Information
  public DifferentialDriveWheelSpeeds getDriveWheelSpeeds() {
    // returns wheel speeds in m/s (meters per second)
    double leftRotationsPerSecond = (double) getLeftEncoderVelocity() / kEncoderResolution / kGearRatio * 10;        
    double leftVelocity = leftRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(kWheelRadius);
    double rightRotationsPerSecond = (double) getRightEncoderVelocity() / kEncoderResolution / kGearRatio * 10;        
    double rightVelocity = rightRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(kWheelRadius);
    return new DifferentialDriveWheelSpeeds(leftVelocity, rightVelocity);
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees((gyroReversed) ? -gyro.getAngle() : gyro.getAngle());
  }

  public double getLeftWheelDistance() {
    double leftDistance = ((double) getLeftEncoderPosition()) / kEncoderResolution / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadius);
    return leftDistance;
  }

  public double getRightWheelDistance() {
    double rightDistance = ((double) getRightEncoderPosition()) / kEncoderResolution / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadius);
    return rightDistance;
  }

  public double getLeftEncoderPosition() {
    return frontLeftMotor.getSelectedSensorPosition();
  }

  public double getRightEncoderPosition() {
    return frontRightMotor.getSelectedSensorPosition();
  }

  public double getLeftEncoderVelocity() {
    return frontLeftMotor.getSelectedSensorVelocity();
  }

  public double getRightEncoderVelocity() {
    return frontRightMotor.getSelectedSensorVelocity();
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  public SimpleMotorFeedforward getSimpleMotorFeedForward() {
    return feedForward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public double getGyroAngle() {
    return gyroReversed ? -gyro.getAngle() : gyro.getAngle();
  }

  // Reset Functions
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    resetGyro();
    System.out.println("WARNING! " + getHeading());
    odometry.resetPosition(pose, getHeading());
  }

  public void resetEncoders() {
    frontLeftMotor.setSelectedSensorPosition(0);
    frontRightMotor.setSelectedSensorPosition(0);
  }

  public void resetGyro() {
    gyro.reset();
  }

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
