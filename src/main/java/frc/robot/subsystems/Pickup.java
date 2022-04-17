package frc.robot.subsystems;

import frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pickup extends SubsystemBase {
  private final WPI_TalonFX pickup;

  private double speed = PickupConstants.kPickupSpeed;
  
  /** Creates a new Intake. */
  public Pickup() {
    pickup = new WPI_TalonFX(PickupConstants.kPickupMotorID);
    pickup.configFactoryDefault();
    pickup.setInverted(PickupConstants.kPickupInverted);
  }

  public void pickup() {
    pickup.set(speed);
  }

  public void stop() {
    pickup.set(0);
  }

  public void reverse() {
    pickup.set(-speed);
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }

  public double getSpeedValue() {
    return speed;
  }

  // Returns velocity in rotations/s
  public double getPickupVelocity() {
    return pickup.getSelectedSensorVelocity() / 2048.0 * 10.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RawPickupVelocity", pickup.getSelectedSensorVelocity());
    SmartDashboard.putNumber("PickupVelocityInRPS", getPickupVelocity());
  }
}
