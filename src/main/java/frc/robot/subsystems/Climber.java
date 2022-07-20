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

/**
 * Climber class that consists of two motors and a slightly complicated means of control.
 * 
 * <p> Few notes about the climber. First: Because each climber attached to the robot would
 * essentially be "opposite" to one another, at least in terms of where the motor would be
 * positioned, there is only one constant to specify the direction of the climber. Whatever
 * direction is specified for the Master motor in {@link frc.robot.Constants} will be flipped
 * for the follower motor in this two motor climber assembly. Secondly, this climber uses a PID
 * positional loop for raising and lowering itself, which allows for fast climbing, but creates a
 * requirement for the climber to be COMPLETELY LOWERED when the code is first started up on
 * the RoboRIO. Otherwise, you will run into the issue of overextending the climber. Be careful
 * when working with this and tweaking numbers, it is very easy to have the climbers overretract
 * too. These are hard limits that are set so you will need to tweak values in @link frc.robot.Constants}
 * if you need to extend or retract further in any direction.
 * 
 * <p> As for that mention of the "complicated means of control", this is because rather than having
 * a straightforward {@code raise()} and {@code lower()} function, I have instead opted to have
 * speeds be set manually, and then have a run command in order to then have the climber move
 * according to the previously specified speed. I believe the reason for this was to dynamically
 * change speed if needed during competition, to have control of the speed within {@link
 * frc.robot.commands.SubsystemSpeedSetter}, and to have the limits be properly set within the positional
 * PID loops, but other than that, the subsystem works as intended and can raise and retract normally.
 */
public class Climber extends SubsystemBase {
  private final WPI_TalonFX masterMotor, followerMotor;
  private double speed;

  /** Creates a new Climber subsystem */
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

  /**
   * Make the climber move, with the current specified speed that's set, enforcing
   * proper positional limits as required depended on if we're extending or climbing.
   */
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

  /**
   * Immediately stops the climber
   */
  public void stop() {
    masterMotor.set(0);
    followerMotor.set(0);
  }

  /**
   * Set the climber to its extending speed
   */
  public void setToExtend() {
    speed = ClimberConstants.kExtensionSpeed;
  }

  /**
   * Set the climber to its climbing speed
   */
  public void setToClimb() {
    speed = ClimberConstants.kClimbingSpeed;
  }

  /**
   * Set a manual speed to the climber. I don't think this actually works because of how {@code run()}
   * is implemented, but the method exists if you want to set a speed and extend the functionality of
   * this subsystem.
   * @param speed output speed to set as a percentage
   */
  public void setSpeed(double speed) {
    this.speed = speed;
  }

  /**
   * Get the current speed of the climber
   * @return Current speed of the climber as its stored value, NOT the physical current running speed
   */
  public double getSpeedValue() {
    return speed;
  }

  /**
   * Pushes current positional information of the climber to ShuffleBoard
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RawClimberMasterPosition", masterMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("RawClimberFollowerPosition", followerMotor.getSelectedSensorPosition());
  }
}
