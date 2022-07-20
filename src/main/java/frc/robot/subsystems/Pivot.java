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

/**
 * Pivot class that consists of a single motor that aids in extending the {@link Pickup} from
 * the perimeter of the robot frame.
 * 
 * <p> This is a fairly simple subsystem that utilizes apositional PID loop, although
 * during competition I will note we had to tweak the raised and lowered position a few times because
 * of how badly I was abusing the {@link Pickup}. The important thing here is understanding how much
 * further up and down you need to position the {@code Pivot} so that it is at the right height for
 * picking up power cells, all adjustable in the {@link frc.robot.Constants} class. Something that I
 * had thought about is setting the {@code Pivot} to {@code BrakeMode}, although I have a strong
 * feeling this would negative impact performance, as the {@code Pivot} would become very rigid as a result.
 * Keep the rigidity of the subsystem in mind if you ever need to create a similar subsystem in the
 * future and think about whether or not you need the flexibility of {@code CoastMode} or more
 * restricted movement with {@code BrakeMode}.
 */
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

  /**
   * Lowers the {@link Pickup} to the front of the robot towards the ground
   */
  public void drop() {
    pivotMotor.set(TalonFXControlMode.Position, PivotConstants.kLoweredPose);
  }

  /**
   * Raises the {@code Pickup} back within the robot frame perimeter, with a
   * bit of leeway so we don't break anything
   */
  public void raise() {
    pivotMotor.set(TalonFXControlMode.Position, PivotConstants.kRaisedPose);
  }

  /**
   * Immediately stops the pivot from moving
   */
  public void stop() {
    pivotMotor.set(0);
  }

  /**
   * Pushes current positional information of the pivot to ShuffleBoard
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Position", pivotMotor.getSelectedSensorPosition());
  }
}
