package frc.robot.subsystems;

import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * Indexer class that consists of a single motor that transports power cells from the {@link Pickup}
 * to the {@link Shooter}.
 * 
 * <p> This is another simple subsystem similar to that of the {@link Pickup}, using a bang-bang control
 * scheme with reversed functionality if required. Change numbers in {@link frc.robot.Constants} as needed
 * if new speeds or directions are required.
 */
public class Indexer extends SubsystemBase{
    private final WPI_TalonFX indexer;

    private double speed = IndexerConstants.kIndexerSpeed;

    /**
     * Creates a new Indexer
     */
    public Indexer() {
        indexer = new WPI_TalonFX(IndexerConstants.kIndexerMotorID);
        indexer.configFactoryDefault();
        indexer.setInverted(IndexerConstants.kIndexerInverted);
    }

    /**
     * Transport power cells from the pickup to the shooter
     */
    public void index() {
        indexer.set(speed);
    }

    /**
     * Immedately stops the indexer from indexing
     */
    public void stop() {
        indexer.set(0);
    }

    /**
     * Transport power cells towards the pickup to eject
     */
    public void reverse() {
        indexer.set(-speed);
    }

    /**
     * Set a manual speed for the indexer
     * @param speed output speed to set as a percentage
     */
    public void setSpeed(double speed) {
        this.speed = speed;
    }

    /**
     * Get the current speed of the indexer
     * @return Curent speed of the indexer as a stored value, NOT the physical current running speed
     */
    public double getSpeedValue() {
        return speed;
    }

    /**
     * Get the current velocity of the indexer in rot/s
     * @return Current physical indexer velocity (NOT the stored value)
     */
    public double getIndexerVelocity() {
        return indexer.getSelectedSensorVelocity() / 2048.0 * 10.0;
    }

    /**
     * Pushes current velocity information of the indexer to ShuffleBoard
     */
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("RawIndexerVelocity", indexer.getSelectedSensorVelocity());
        SmartDashboard.putNumber("IndexerVelocityInRPS", getIndexerVelocity());
    }
}
