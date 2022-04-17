// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final WPI_TalonFX masterMotor, followerMotor;
  private double speed;

  /** Creates a new Climber. */
  public Climber() {
    masterMotor = new WPI_TalonFX(ClimberConstants.kMasterMotorID);
    followerMotor = new WPI_TalonFX(ClimberConstants.kFollowerMotorID);
    masterMotor.configFactoryDefault();
    followerMotor.configFactoryDefault();
    masterMotor.setInverted(ClimberConstants.kIsMasterInverted);
    followerMotor.setInverted(ClimberConstants.kIsMasterInverted);
    speed = ClimberConstants.kExtensionSpeed;
    masterMotor.setNeutralMode(NeutralMode.Brake);
    followerMotor.setNeutralMode(NeutralMode.Brake);

    // PID stuff
    masterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, ClimberConstants.kSlotIdx, ClimberConstants.kTimeoutMs);
    masterMotor.config_kF(ClimberConstants.kSlotIdx, ClimberConstants.kF);
    masterMotor.config_kP(ClimberConstants.kSlotIdx, ClimberConstants.kP);
    masterMotor.config_kI(ClimberConstants.kSlotIdx, ClimberConstants.kI);
    masterMotor.config_kD(ClimberConstants.kSlotIdx, ClimberConstants.kD);
    followerMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, ClimberConstants.kSlotIdx, ClimberConstants.kTimeoutMs);
    followerMotor.config_kF(ClimberConstants.kSlotIdx, ClimberConstants.kF);
    followerMotor.config_kP(ClimberConstants.kSlotIdx, ClimberConstants.kP);
    followerMotor.config_kI(ClimberConstants.kSlotIdx, ClimberConstants.kI);
    followerMotor.config_kD(ClimberConstants.kSlotIdx, ClimberConstants.kD);
  }

  public void run() {
    if (speed == ClimberConstants.kExtensionSpeed) {
      masterMotor.set(ControlMode.Position, ClimberConstants.kExtendLimit);
      followerMotor.set(ControlMode.Position, ClimberConstants.kExtendLimit);
    }
    if (speed == ClimberConstants.kClimbingSpeed) {
      masterMotor.set(ControlMode.Position, ClimberConstants.kRetractLimit);
      followerMotor.set(ControlMode.Position, ClimberConstants.kRetractLimit);
    }
  }

  public void stop() {
    masterMotor.set(0);
    followerMotor.set(0);
  }

  public void setToExtend() {
    speed = ClimberConstants.kExtensionSpeed;
  }

  public void setToClimb() {
    speed = ClimberConstants.kClimbingSpeed;
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }

  public double getSpeedValue() {
    return speed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RawClimberMasterPosition", masterMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("RawClimberFollowerPosition", followerMotor.getSelectedSensorPosition());
  }
}
