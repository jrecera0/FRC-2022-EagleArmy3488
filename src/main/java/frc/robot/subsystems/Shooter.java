package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.*;

/**
 * Shooter class that consists for two motors and a janky velocity PID loop.
 * 
 * <p> Similar to other subsystems, because of its assembly, depending on what direction the master
 * motor is set to, the follower motor will always rotate opposite to that. Originally, this subsystem
 * was designed with the expectation of us possibly shooting the low goal after tweaking the PID loop
 * and figuring out speeds, but this never actually happened, so the method name is a little arbritrary.
 * In reality, the implementation ended up being "high goal" and "a little slower than high goal" if for
 * whatever reason we started overshooting. Just keep this in mind in case you're looking at the method
 * names and expecting one to be drastically slower than the other, because this is NOT the case. The
 * other main thing to note about this subsystem is the fact that the PID loop is... definitely characterized
 * wrong. For competition, it worked, but in terms of getting an accurate readout of the speed of the wheels
 * themselves, it seems very innaccurate and is in need of tweaking. So although on paper, the target speeds
 * specified should be in m/s, I doubt this is actually the case and thus, this very weird characterization
 * is what lead to the weird thresholding values for the shooter to be flagged as "ready".
 * Just be weary when looking at velocity information coming from this subsystem and know that the thresholds
 * have more or less been confirmed to be known working values for shooting into the high goal.
 * 
 * <p> Other important note: Shooting with the power cells we own gets weird because as they become deflated,
 * the way they shoot changes pretty signficantly. This was not an issue down on the field with field elements
 * being replaced often, but this should be noted if you ever run this robot at school and run into some
 * inconsistencies.
 */
public class Shooter extends SubsystemBase {
    private final WPI_TalonFX followerMotor, masterMotor;

    private final PIDController pidController;
    private final SimpleMotorFeedforward feedForward;

    private double speed;
    private double thresh;

    /**
     * Creates a new Shooter subsystem
     */
    public Shooter() {
        // Initialize motors
        masterMotor = new WPI_TalonFX(ShooterConstants.kFollowerMotorID);
        followerMotor = new WPI_TalonFX(ShooterConstants.kMasterMotorID);
        masterMotor.configFactoryDefault();
        followerMotor.configFactoryDefault();
        masterMotor.setInverted(ShooterConstants.kFrontShooterMotorInverted);
        followerMotor.setInverted(!ShooterConstants.kFrontShooterMotorInverted);
        followerMotor.follow(masterMotor);

        // Setup controllers for optimized shooting
        pidController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
        feedForward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);

        // Set default speeds
        speed = ShooterConstants.kHighGoalSpeed;
        thresh = ShooterConstants.kHighGoalThresh;
    }

    /**
     * Sets the voltage of the shooter motors to shooting speed
     */
    public void shoot() {
        masterMotor.setVoltage(feedForward.calculate(speed) + pidController.calculate(getVelocity(), speed));
    }

    /**
     * Immedately stops the shooter
     */
    public void stop() {
        masterMotor.set(0);
    }

    /**
     * Sets the internal speed and threshold for the shooter for High Goal shooting
     */
    public void setToHighGoal() {
        speed = ShooterConstants.kHighGoalSpeed;
        thresh = ShooterConstants.kHighGoalThresh;
    }

    /**
     * Sets the internal speed and threshold for the shooter for "Low Goal" shooting (which
     * really isn't low goal but is more or less just a slightly slower speed, unless {@link
     * frc.robot.Constants} is changed.
     */
    public void setToLowGoal() {
        speed = ShooterConstants.kLowGoalSpeed;
        thresh = ShooterConstants.kLowGoalThresh;
    }

    /**
     * Set a manual speed to the shooter. Note that this is not as a percentage!
     * Also I have no idea if this works because as of writing, there is no way to set a threshold
     * manually, so you may run into issues when attempting to set a speed in this way.
     * @param speed outspeed speed to set internally in "m/s"
     */
    public void setSpeed(double speed) {
        this.speed = speed;
    }

    /**
     * Get the current speed of the shooter
     * @return Current speed of the shooter as its stored value, NOT the physical current running speed
     */
    public double getSpeedValue() {
        return speed;
    }

    /**
     * Get the current threshold speed value of the shooter. This is so that the
     * {@link Indexer} knows when to start moving the power cells to the shooter!
     * @return Current threshold value of the shooter to count as "ready"
     */
    public double getThreshValue() {
        return thresh;
    }

    /**
     * Gets the current physical running velocity of the shooter. Note that this recalculates
     * the velocity from the motors to m/s with gear ratio and diameter taken into account.
     * @return Velocity of the shooter in m/s
     */
    public double getVelocity() {
        return
            (double) masterMotor.getSelectedSensorVelocity() / ShooterConstants.kEncoderResolution /
            ShooterConstants.kGearRatio * 10 * Math.PI * Units.inchesToMeters(ShooterConstants.kWheelDiameter);
    }

    /**
     * Pushes current velocity information of the shooter to ShuffleBoard
     */
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("RawMasterShooterVelocity", masterMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber(("RawFollowerShooterVelocity"), followerMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("ShooterVelocityInM/S", getVelocity());
    }
}
