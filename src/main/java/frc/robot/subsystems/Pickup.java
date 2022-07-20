package frc.robot.subsystems;

import frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Pickup class that consists of a single motor that either intakes or outtakes power cells.
 * 
 * <p> This is a very, VERY basic subsystem with a bang-bang control scheme to either pick up
 * or outtake balls, depending on the method used. Use {@link frc.robot.Constants} to set corresponding
 * speeds and directional values to this subsystem as need be.
 */
public class Pickup extends SubsystemBase {
  private final WPI_TalonFX pickup;

  private double speed = PickupConstants.kPickupSpeed;
  
  /** Creates a new Intake. */
  public Pickup() {
    pickup = new WPI_TalonFX(PickupConstants.kPickupMotorID);
    pickup.configFactoryDefault();
    pickup.setInverted(PickupConstants.kPickupInverted);
  }

  /**
   * Rotate the pickup motor to pick up power cells from the field
   */
  public void pickup() {
    pickup.set(speed);
  }

  /**
   * Immediately stops the pickup from moving
   */
  public void stop() {
    pickup.set(0);
  }

  /**
   * Reverse the movement of the pickup in case we need to remove a power cell
   * from the robot
   */
  public void reverse() {
    pickup.set(-speed);
  }

  /**
   * Set a manual speed to the pickup to use
   * @param speed output speed to set as a percentage
   */
  public void setSpeed(double speed) {
    this.speed = speed;
  }

  /**
   * Get the current speed of the pickup
   * @return Current speed of the pickup as its stored value, NOT the physical current running speed
   */
  public double getSpeedValue() {
    return speed;
  }

  /**
   * Get the current velocity of the pickup in rot/s
   * @return Current physical pickup velocity (NOT the stored value)
   */
  public double getPickupVelocity() {
    return pickup.getSelectedSensorVelocity() / 2048.0 * 10.0;
  }

  /**
   * Pushes current velocity information of the pickup to ShuffleBoard
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RawPickupVelocity", pickup.getSelectedSensorVelocity());
    SmartDashboard.putNumber("PickupVelocityInRPS", getPickupVelocity());
  }
}
