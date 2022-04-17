// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
  private final WPI_TalonFX pivotMotor;

  /** Creates a new Pivot. */
  public Pivot() {
    pivotMotor = new WPI_TalonFX(PivotConstants.kPivotMotorID);
    pivotMotor.configFactoryDefault();
    pivotMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PivotConstants.kSlotIdx, PivotConstants.kTimeoutMs);
    pivotMotor.config_kF(PivotConstants.kSlotIdx, PivotConstants.kF);
    pivotMotor.config_kP(PivotConstants.kSlotIdx, PivotConstants.kP);
    pivotMotor.config_kI(PivotConstants.kSlotIdx, PivotConstants.kI);
    pivotMotor.config_kD(PivotConstants.kSlotIdx, PivotConstants.kD);
  }

  public void drop() {
    pivotMotor.set(TalonFXControlMode.Position, PivotConstants.kLoweredPose);
  }

  public void raise() {
    pivotMotor.set(TalonFXControlMode.Position, PivotConstants.kRaisedPose);
  }

  public void stop() {
    pivotMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Position", pivotMotor.getSelectedSensorPosition());
  }
}
